package org.cocos2dx.demo;

import android.app.NativeActivity;
import android.graphics.PixelFormat;
import android.os.Bundle;

public class TranslucentDemo3x extends NativeActivity{

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		
		getWindow().setFormat(PixelFormat.RGBA_8888);
	}
}
