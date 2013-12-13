/****************************************************************************
Copyright (c) 2010-2011 cocos2d-x.org

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
package org.cocos.demo;

import org.cocos2dx.lib.Cocos2dxActivity;
import org.cocos2dx.lib.Cocos2dxGLSurfaceView;

import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.FrameLayout;

public class SecondActivityTest extends Cocos2dxActivity{
	
    protected void onCreate(Bundle savedInstanceState){
		super.onCreate(savedInstanceState);	
		
		Button btnBack = new Button(this);
		btnBack.setText("back");
		btnBack.setTextColor(Color.RED);
		ViewGroup.LayoutParams btnBackParams = new ViewGroup.LayoutParams(
				ViewGroup.LayoutParams.WRAP_CONTENT,ViewGroup.LayoutParams.WRAP_CONTENT);
		framelayout.addView(btnBack,btnBackParams);	
		
		btnBack.setOnClickListener(listener);
		
		Button btnQuit = new Button(this);
		btnQuit.setText("quit");
		btnQuit.setTextColor(Color.RED);
		FrameLayout.LayoutParams btnQuitParams = new FrameLayout.LayoutParams(
				FrameLayout.LayoutParams.WRAP_CONTENT,FrameLayout.LayoutParams.WRAP_CONTENT);
		btnQuitParams.topMargin = 100;
		framelayout.addView(btnQuit,btnQuitParams);	
		
		btnQuit.setOnClickListener(quitListener);
	}
    
    OnClickListener listener = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			TwoCocosActivityTest.nativeEnd();
			SecondActivityTest.this.finish();			
			
			
			Intent intent = new Intent(SecondActivityTest.this, TwoCocosActivityTest.class);
			startActivity(intent);
		}

	};
	
	OnClickListener quitListener = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			SecondActivityTest.this.finish();
			android.os.Process.killProcess(android.os.Process.myPid());
		}
	};

    public Cocos2dxGLSurfaceView onCreateView() {
    	Cocos2dxGLSurfaceView glSurfaceView = new Cocos2dxGLSurfaceView(this);
    	// TwoCocosActivityTest should create stencil buffer
    	glSurfaceView.setEGLConfigChooser(5, 6, 5, 0, 16, 8);
    	
    	return glSurfaceView;
    } 
}
