package org.cocos2dx.demo;

import android.app.NativeActivity;
import android.graphics.Color;
import android.graphics.PixelFormat;
import android.os.Bundle;
import android.view.WindowManager;
import android.widget.RelativeLayout;
import android.widget.TextView;

public class TranslucentDemo3x extends NativeActivity{

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		
		getWindow().setFormat(PixelFormat.RGBA_8888);
	}
	
	@Override
	public void onAttachedToWindow() {
		RelativeLayout relativeLayout = new RelativeLayout(this);	
		WindowManager.LayoutParams wParams = new WindowManager.LayoutParams(WindowManager.LayoutParams.WRAP_CONTENT, 
				WindowManager.LayoutParams.WRAP_CONTENT,
				WindowManager.LayoutParams.FIRST_APPLICATION_WINDOW, 40, PixelFormat.TRANSLUCENT);
		
		RelativeLayout.LayoutParams rParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.WRAP_CONTENT
				,RelativeLayout.LayoutParams.WRAP_CONTENT);			
		TextView textView = new TextView(this);
		textView.setTextSize(36);
		textView.setTextColor(Color.RED);
		textView.setText("Android Button");
		relativeLayout.addView(textView,rParams);
		
		getWindowManager().addView(relativeLayout,wParams);
		
		super.onAttachedToWindow();
	}
}
