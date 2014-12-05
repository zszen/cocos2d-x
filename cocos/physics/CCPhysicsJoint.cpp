/****************************************************************************
 Copyright (c) 2013 Chukong Technologies Inc.
 
 http://www.cocos2d-x.org
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/

#include "physics/CCPhysicsJoint.h"
#if CC_USE_PHYSICS
#include "chipmunk.h"

#include "physics/CCPhysicsBody.h"
#include "physics/CCPhysicsWorld.h"

#include "chipmunk/CCPhysicsJointInfo_chipmunk.h"
#include "chipmunk/CCPhysicsBodyInfo_chipmunk.h"
#include "chipmunk/CCPhysicsShapeInfo_chipmunk.h"
#include "chipmunk/CCPhysicsHelper_chipmunk.h"
#include "2d/CCNode.h"

NS_CC_BEGIN

PhysicsJoint::PhysicsJoint()
: _bodyA(nullptr)
, _bodyB(nullptr)
, _world(nullptr)
, _info(nullptr)
, _enable(false)
, _collisionEnable(true)
, _destoryMark(false)
, _initDirty(true)
, _tag(0)
{
    
}

PhysicsJoint::~PhysicsJoint()
{
    // reset the shapes collision group
    setCollisionEnable(true);
    
    CC_SAFE_DELETE(_info);
}

bool PhysicsJoint::init(cocos2d::PhysicsBody *a, cocos2d::PhysicsBody *b)
{
    do
    {
        CCASSERT(a != nullptr && b != nullptr, "the body passed in is nil");
        CCASSERT(a != b, "the two bodies are equal");
        
        CC_BREAK_IF(!(_info = new (std::nothrow) PhysicsJointInfo(this)));
        
        _bodyA = a;
        
        _bodyB = b;
        
        
        return true;
    } while (false);
    
    return false;
}

bool PhysicsJoint::initJoint()
{
    _bodyA->_joints.push_back(this);
    _bodyB->_joints.push_back(this);
}

void PhysicsJoint::setEnable(bool enable)
{
    if (_enable != enable)
    {
        _enable = enable;
        
        if (_world != nullptr)
        {
            if (enable)
            {
                _world->addJointOrDelay(this);
            }else
            {
                _world->removeJointOrDelay(this);
            }
        }
    }
}

PhysicsBodyInfo* PhysicsJoint::getBodyInfo(PhysicsBody* body) const
{
    return body->_info;
}

Node* PhysicsJoint::getBodyNode(PhysicsBody* body) const
{
    return body->_node;
}

void PhysicsJoint::setCollisionEnable(bool enable)
{
    if (_collisionEnable != enable)
    {
        _collisionEnable = enable;
    }
}

void PhysicsJoint::removeFormWorld()
{
    if (_world)
    {
        _world->removeJoint(this, false);
    }
}

void PhysicsJoint::destroy(PhysicsJoint* joint)
{
    if (joint)
    {
        // remove the joint and delete it.
        if (joint->_world)
        {
            joint->_world->removeJoint(joint, true);
        }
        else
        {
            if (joint->_bodyA)
            {
                joint->_bodyA->removeJoint(joint);
            }
            
            if (joint->_bodyB)
            {
                joint->_bodyB->removeJoint(joint);
            }
            
            delete joint;
        }
    }
}

void PhysicsJoint::setMaxForce(float force)
{
    for (auto joint : _info->getJoints())
    {
        joint->maxForce = PhysicsHelper::float2cpfloat(force);
    }
}

float PhysicsJoint::getMaxForce() const
{
    return PhysicsHelper::cpfloat2float(_info->getJoints().front()->maxForce);
}

PhysicsJointFixed* PhysicsJointFixed::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr)
{
    auto joint = new (std::nothrow) PhysicsJointFixed();
    
    if (joint && joint->init(a, b))
    {
        joint->_anchr = anchr;
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointFixed::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        getBodyNode(_bodyA)->setPosition(_anchr);
        getBodyNode(_bodyB)->setPosition(_anchr);
        
        // add a pivot joint to fixed two body together
        auto joint = cpPivotJointNew(getBodyInfo(_bodyA)->getBody(), getBodyInfo(_bodyB)->getBody(),
                                        PhysicsHelper::point2cpv(_anchr));
        CC_BREAK_IF(joint == nullptr);
        _info->add(joint);
        
        // add a gear joint to make two body have the same rotation.
        joint = cpGearJointNew(getBodyInfo(_bodyA)->getBody(), getBodyInfo(_bodyB)->getBody(), 0, 1);
        CC_BREAK_IF(joint == nullptr);
        _info->add(joint);
        
        setCollisionEnable(false);
        
        return true;
    } while (false);
    
    return false;
}

PhysicsJointPin* PhysicsJointPin::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr)
{
    auto joint = new (std::nothrow) PhysicsJointPin();
    
    if (joint && joint->init(a, b))
    {
        joint->_anchr = anchr;
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointPin::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    
    do
    {
        auto joint = cpPivotJointNew(getBodyInfo(_bodyA)->getBody(), getBodyInfo(_bodyB)->getBody(),
                                       PhysicsHelper::point2cpv(_anchr));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

PhysicsJointLimit* PhysicsJointLimit::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr1, const Vec2& anchr2, float min, float max)
{
    auto joint = new (std::nothrow) PhysicsJointLimit();
    
    if (joint && joint->init(a, b))
    {
        joint->_anchr1 = anchr1;
        joint->_anchr2 = anchr2;
        joint->_min = min;
        joint->_max = max;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

PhysicsJointLimit* PhysicsJointLimit::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr1, const Vec2& anchr2)
{
    return construct(a, b, anchr1, anchr2, 0, b->local2World(anchr1).getDistance(a->local2World(anchr2)));
}

bool PhysicsJointLimit::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpSlideJointNew(getBodyInfo(_bodyA)->getBody(), getBodyInfo(_bodyB)->getBody(),
                                       PhysicsHelper::point2cpv(_anchr1),
                                       PhysicsHelper::point2cpv(_anchr2),
                                       PhysicsHelper::float2cpfloat(_min),
                                       PhysicsHelper::float2cpfloat(_max));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointLimit::getMin() const
{
    return PhysicsHelper::cpfloat2float(cpSlideJointGetMin(_info->getJoints().front()));
}

void PhysicsJointLimit::setMin(float min)
{
    cpSlideJointSetMin(_info->getJoints().front(), PhysicsHelper::float2cpfloat(min));
}

float PhysicsJointLimit::getMax() const
{
    return PhysicsHelper::cpfloat2float(cpSlideJointGetMax(_info->getJoints().front()));
}

void PhysicsJointLimit::setMax(float max)
{
    cpSlideJointSetMax(_info->getJoints().front(), PhysicsHelper::float2cpfloat(max));
}

Vec2 PhysicsJointLimit::getAnchr1() const
{
    return PhysicsHelper::cpv2point(cpSlideJointGetAnchr1(_info->getJoints().front()));
}

void PhysicsJointLimit::setAnchr1(const Vec2& anchr)
{
    cpSlideJointSetAnchr1(_info->getJoints().front(), PhysicsHelper::point2cpv(anchr));
}

Vec2 PhysicsJointLimit::getAnchr2() const
{
    return PhysicsHelper::cpv2point(cpSlideJointGetAnchr2(_info->getJoints().front()));
}

void PhysicsJointLimit::setAnchr2(const Vec2& anchr)
{
    cpSlideJointSetAnchr1(_info->getJoints().front(), PhysicsHelper::point2cpv(anchr));
}

PhysicsJointDistance* PhysicsJointDistance::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr1, const Vec2& anchr2)
{
    auto joint = new (std::nothrow) PhysicsJointDistance();
    
    if (joint && joint->init(a, b))
    {
        joint->_anchr1 = anchr1;
        joint->_anchr2 = anchr2;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointDistance::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpPinJointNew(getBodyInfo(_bodyA)->getBody(),
                                            getBodyInfo(_bodyB)->getBody(),
                                            PhysicsHelper::point2cpv(_anchr1),
                                            PhysicsHelper::point2cpv(_anchr2));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointDistance::getDistance() const
{
    return PhysicsHelper::cpfloat2float(cpPinJointGetDist(_info->getJoints().front()));
}

void PhysicsJointDistance::setDistance(float distance)
{
    cpPinJointSetDist(_info->getJoints().front(), PhysicsHelper::float2cpfloat(distance));
}

PhysicsJointSpring* PhysicsJointSpring::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& anchr1, const Vec2& anchr2, float stiffness, float damping)
{
    auto joint = new (std::nothrow) PhysicsJointSpring();
    
    if (joint && joint->init(a, b))
    {
        joint->_anchr1 = anchr1;
        joint->_anchr2 = anchr2;
        joint->_stiffness = stiffness;
        joint->_damping = damping;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointSpring::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do {
        auto joint = cpDampedSpringNew(getBodyInfo(_bodyA)->getBody(),
                                                getBodyInfo(_bodyB)->getBody(),
                                                PhysicsHelper::point2cpv(_anchr1),
                                                PhysicsHelper::point2cpv(_anchr2),
                                                PhysicsHelper::float2cpfloat(_bodyB->local2World(_anchr1).getDistance(_bodyA->local2World(_anchr2))),
                                                PhysicsHelper::float2cpfloat(_stiffness),
                                                PhysicsHelper::float2cpfloat(_damping));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

Vec2 PhysicsJointSpring::getAnchr1() const
{
    return PhysicsHelper::cpv2point(cpDampedSpringGetAnchr1(_info->getJoints().front()));
}

void PhysicsJointSpring::setAnchr1(const Vec2& anchr)
{
    cpDampedSpringSetAnchr1(_info->getJoints().front(), PhysicsHelper::point2cpv(anchr));
}

Vec2 PhysicsJointSpring::getAnchr2() const
{
    return PhysicsHelper::cpv2point(cpDampedSpringGetAnchr2(_info->getJoints().front()));
}

void PhysicsJointSpring::setAnchr2(const Vec2& anchr)
{
    cpDampedSpringSetAnchr1(_info->getJoints().front(), PhysicsHelper::point2cpv(anchr));
}

float PhysicsJointSpring::getRestLength() const
{
    return PhysicsHelper::cpfloat2float(cpDampedSpringGetRestLength(_info->getJoints().front()));
}

void PhysicsJointSpring::setRestLength(float restLength)
{
    cpDampedSpringSetRestLength(_info->getJoints().front(), PhysicsHelper::float2cpfloat(restLength));
}

float PhysicsJointSpring::getStiffness() const
{
    return PhysicsHelper::cpfloat2float(cpDampedSpringGetStiffness(_info->getJoints().front()));
}

void PhysicsJointSpring::setStiffness(float stiffness)
{
    cpDampedSpringSetStiffness(_info->getJoints().front(), PhysicsHelper::float2cpfloat(stiffness));
}

float PhysicsJointSpring::getDamping() const
{
    return PhysicsHelper::cpfloat2float(cpDampedSpringGetDamping(_info->getJoints().front()));
}

void PhysicsJointSpring::setDamping(float damping)
{
    cpDampedSpringSetDamping(_info->getJoints().front(), PhysicsHelper::float2cpfloat(damping));
}

PhysicsJointGroove* PhysicsJointGroove::construct(PhysicsBody* a, PhysicsBody* b, const Vec2& grooveA, const Vec2& grooveB, const Vec2& anchr2)
{
    auto joint = new (std::nothrow) PhysicsJointGroove();
    
    if (joint && joint->init(a, b))
    {
        joint->_grooveA = grooveA;
        joint->_grooveB = grooveB;
        joint->_anchr2 = anchr2;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointGroove::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do {
        auto joint = cpGrooveJointNew(getBodyInfo(_bodyA)->getBody(),
                                               getBodyInfo(_bodyB)->getBody(),
                                               PhysicsHelper::point2cpv(_grooveA),
                                               PhysicsHelper::point2cpv(_grooveB),
                                               PhysicsHelper::point2cpv(_anchr2));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

Vec2 PhysicsJointGroove::getGrooveA() const
{
    return PhysicsHelper::cpv2point(cpGrooveJointGetGrooveA(_info->getJoints().front()));
}

void PhysicsJointGroove::setGrooveA(const Vec2& grooveA)
{
    cpGrooveJointSetGrooveA(_info->getJoints().front(), PhysicsHelper::point2cpv(grooveA));
}

Vec2 PhysicsJointGroove::getGrooveB() const
{
    return PhysicsHelper::cpv2point(cpGrooveJointGetGrooveB(_info->getJoints().front()));
}

void PhysicsJointGroove::setGrooveB(const Vec2& grooveB)
{
    cpGrooveJointSetGrooveB(_info->getJoints().front(), PhysicsHelper::point2cpv(grooveB));
}

Vec2 PhysicsJointGroove::getAnchr2() const
{
    return PhysicsHelper::cpv2point(cpGrooveJointGetAnchr2(_info->getJoints().front()));
}

void PhysicsJointGroove::setAnchr2(const Vec2& anchr2)
{
    cpGrooveJointSetAnchr2(_info->getJoints().front(), PhysicsHelper::point2cpv(anchr2));
}

PhysicsJointRotarySpring* PhysicsJointRotarySpring::construct(PhysicsBody* a, PhysicsBody* b, float stiffness, float damping)
{
    auto joint = new (std::nothrow) PhysicsJointRotarySpring();
    
    if (joint && joint->init(a, b))
    {
        joint->_stiffness = stiffness;
        joint->_damping = damping;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointRotarySpring::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do {
        auto joint = cpDampedRotarySpringNew(getBodyInfo(_bodyA)->getBody(),
                                                      getBodyInfo(_bodyB)->getBody(),
                                                      PhysicsHelper::float2cpfloat(_bodyB->getRotation() - _bodyA->getRotation()),
                                                      PhysicsHelper::float2cpfloat(_stiffness),
                                                      PhysicsHelper::float2cpfloat(_damping));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointRotarySpring::getRestAngle() const
{
    return PhysicsHelper::cpfloat2float(cpDampedRotarySpringGetRestAngle(_info->getJoints().front()));
}

void PhysicsJointRotarySpring::setRestAngle(float restAngle)
{
    cpDampedRotarySpringSetRestAngle(_info->getJoints().front(), PhysicsHelper::float2cpfloat(restAngle));
}

float PhysicsJointRotarySpring::getStiffness() const
{
    return PhysicsHelper::cpfloat2float(cpDampedRotarySpringGetStiffness(_info->getJoints().front()));
}

void PhysicsJointRotarySpring::setStiffness(float stiffness)
{
    cpDampedRotarySpringSetStiffness(_info->getJoints().front(), PhysicsHelper::float2cpfloat(stiffness));
}

float PhysicsJointRotarySpring::getDamping() const
{
    return PhysicsHelper::cpfloat2float(cpDampedRotarySpringGetDamping(_info->getJoints().front()));
}

void PhysicsJointRotarySpring::setDamping(float damping)
{
    cpDampedRotarySpringSetDamping(_info->getJoints().front(), PhysicsHelper::float2cpfloat(damping));
}

PhysicsJointRotaryLimit* PhysicsJointRotaryLimit::construct(PhysicsBody* a, PhysicsBody* b, float min, float max)
{
    auto joint = new (std::nothrow) PhysicsJointRotaryLimit();
    
    if (joint && joint->init(a, b))
    {
        joint->_min = min;
        joint->_max = max;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

PhysicsJointRotaryLimit* PhysicsJointRotaryLimit::construct(PhysicsBody* a, PhysicsBody* b)
{
    return construct(a, b, 0.0f, 0.0f);
}

bool PhysicsJointRotaryLimit::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpRotaryLimitJointNew(getBodyInfo(_bodyA)->getBody(),
                                                    getBodyInfo(_bodyB)->getBody(),
                                                    PhysicsHelper::float2cpfloat(_min),
                                                    PhysicsHelper::float2cpfloat(_max));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointRotaryLimit::getMin() const
{
    return PhysicsHelper::cpfloat2float(cpRotaryLimitJointGetMin(_info->getJoints().front()));
}

void PhysicsJointRotaryLimit::setMin(float min)
{
    cpRotaryLimitJointSetMin(_info->getJoints().front(), PhysicsHelper::float2cpfloat(min));
}

float PhysicsJointRotaryLimit::getMax() const
{
    return PhysicsHelper::cpfloat2float(cpRotaryLimitJointGetMax(_info->getJoints().front()));
}

void PhysicsJointRotaryLimit::setMax(float max)
{
    cpRotaryLimitJointSetMax(_info->getJoints().front(), PhysicsHelper::float2cpfloat(max));
}

PhysicsJointRatchet* PhysicsJointRatchet::construct(PhysicsBody* a, PhysicsBody* b, float phase, float ratchet)
{
    auto joint = new (std::nothrow) PhysicsJointRatchet();
    
    if (joint && joint->init(a, b))
    {
        joint->_phase = phase;
        joint->_ratchet = ratchet;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointRatchet::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpRatchetJointNew(getBodyInfo(_bodyA)->getBody(),
                                                getBodyInfo(_bodyB)->getBody(),
                                                PhysicsHelper::float2cpfloat(_phase),
                                                PhysicsHelper::cpfloat2float(_ratchet));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointRatchet::getAngle() const
{
    return PhysicsHelper::cpfloat2float(cpRatchetJointGetAngle(_info->getJoints().front()));
}

void PhysicsJointRatchet::setAngle(float angle)
{
    cpRatchetJointSetAngle(_info->getJoints().front(), PhysicsHelper::float2cpfloat(angle));
}

float PhysicsJointRatchet::getPhase() const
{
    return PhysicsHelper::cpfloat2float(cpRatchetJointGetPhase(_info->getJoints().front()));
}

void PhysicsJointRatchet::setPhase(float phase)
{
    cpRatchetJointSetPhase(_info->getJoints().front(), PhysicsHelper::float2cpfloat(phase));
}

float PhysicsJointRatchet::getRatchet() const
{
    return PhysicsHelper::cpfloat2float(cpRatchetJointGetRatchet(_info->getJoints().front()));
}

void PhysicsJointRatchet::setRatchet(float ratchet)
{
    cpRatchetJointSetRatchet(_info->getJoints().front(), PhysicsHelper::float2cpfloat(ratchet));
}

PhysicsJointGear* PhysicsJointGear::construct(PhysicsBody* a, PhysicsBody* b, float phase, float ratio)
{
    auto joint = new (std::nothrow) PhysicsJointGear();
    
    if (joint && joint->init(a, b))
    {
        joint->_phase = phase;
        joint->_ratio = ratio;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointGear::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpGearJointNew(getBodyInfo(_bodyA)->getBody(),
                                             getBodyInfo(_bodyB)->getBody(),
                                             PhysicsHelper::float2cpfloat(_phase),
                                             PhysicsHelper::float2cpfloat(_ratio));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointGear::getPhase() const
{
    return PhysicsHelper::cpfloat2float(cpGearJointGetPhase(_info->getJoints().front()));
}

void PhysicsJointGear::setPhase(float phase)
{
    cpGearJointSetPhase(_info->getJoints().front(), PhysicsHelper::float2cpfloat(phase));
}

float PhysicsJointGear::getRatio() const
{
    return PhysicsHelper::cpfloat2float(cpGearJointGetRatio(_info->getJoints().front()));
}

void PhysicsJointGear::setRatio(float ratio)
{
    cpGearJointSetRatio(_info->getJoints().front(), PhysicsHelper::float2cpfloat(ratio));
}

PhysicsJointMotor* PhysicsJointMotor::construct(PhysicsBody* a, PhysicsBody* b, float rate)
{
    auto joint = new (std::nothrow) PhysicsJointMotor();
    
    if (joint && joint->init(a, b))
    {
        joint->_rate = rate;
        
        return joint;
    }
    
    CC_SAFE_DELETE(joint);
    return nullptr;
}

bool PhysicsJointMotor::initJoint()
{
    if (!_initDirty)
    {
        return true;
    }
    _initDirty = false;
    PhysicsJoint::initJoint();
    do
    {
        auto joint = cpSimpleMotorNew(getBodyInfo(_bodyA)->getBody(),
                                             getBodyInfo(_bodyB)->getBody(),
                                             PhysicsHelper::float2cpfloat(_rate));
        
        CC_BREAK_IF(joint == nullptr);
        
        _info->add(joint);
        
        return true;
    } while (false);
    
    return false;
}

float PhysicsJointMotor::getRate() const
{
    return PhysicsHelper::cpfloat2float(cpSimpleMotorGetRate(_info->getJoints().front()));
}

void PhysicsJointMotor::setRate(float rate)
{
    cpSimpleMotorSetRate(_info->getJoints().front(), PhysicsHelper::float2cpfloat(rate));
}

NS_CC_END
#endif // CC_USE_PHYSICS
