// Copyright 2020 Carnegie Mellon University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package edu.cmu.cs.openfluid;

import android.content.Context;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
// https://stackoverflow.com/questions/5790503/can-we-use-scale-gesture-detector-for-pinch-zoom-in-android

public class SceneScaleGestures implements View.OnTouchListener, GestureDetector.OnGestureListener, ScaleGestureDetector.OnScaleGestureListener {
    private final GestureDetector gesture;
    private final ScaleGestureDetector gestureScale;
    private float scaleFactor = 1.0f;
    private boolean inScale = false;
    private final boolean doubleTab = false;
    private final GabrielClientActivity gabrielClientActivity;

    private float totalDistX = 0.0f;
    private float totalDistY = 0.0f;

    private boolean inScroll = false;

    public SceneScaleGestures (Context c, GabrielClientActivity gabrielClientActivity){
        gesture = new GestureDetector(c, this);
        gestureScale = new ScaleGestureDetector(c, this);
        this.gabrielClientActivity = gabrielClientActivity;
    }

    @Override
    public boolean onTouch(View view, MotionEvent event) {
        int action = event.getActionMasked();

        switch (action) {
            case MotionEvent.ACTION_POINTER_DOWN:
                // When a second pointer is down, it's either a two-finger scroll or a pinch-to-zoom gesture
                if (event.getPointerCount() >= 2) {
                    gabrielClientActivity.setDoubleTouch(true);
                }
                break;
            case MotionEvent.ACTION_POINTER_UP:
                // When a pointer goes up, switch back to single-touch scroll
                gabrielClientActivity.setDoubleTouch(false);

                break;
            default:
//                gabrielClientActivity.setDoubleTouch(false);
        }

        gesture.onTouchEvent(event);
        gestureScale.onTouchEvent(event);
        return true;
    }

    @Override
    public boolean onDown(MotionEvent event) {
        return true;
    }

    @Override
    public boolean onFling(MotionEvent event1, MotionEvent event2, float x, float y) {
        return true;
    }

    @Override
    public void onLongPress(MotionEvent event) {
    }

    @Override
    public boolean onScale(ScaleGestureDetector detector) {
        scaleFactor = detector.getScaleFactor();
//        scaleFactor = (Math.max(scaleFactor, 0.8f)); // prevent our view from becoming too small //
        scaleFactor = ((float)((int)(scaleFactor * 1000))) / 1000; // Change precision to help with jitter when user just rests their fingers //
        gabrielClientActivity.setScaleFactor(scaleFactor);

//        view.setScaleX(scaleFactor);
//        view.setScaleY(scaleFactor);
        onScroll(null, null, 0, 0);
        return true;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector detector) {
        inScale = true;
        return true;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector detector) {
        inScale = false;
        scaleFactor = 1.0f;
        gabrielClientActivity.setScaleFactor(scaleFactor);
        onScroll(null, null, 0, 0);
    }

    @Override
    public boolean onScroll(MotionEvent event1, MotionEvent event2, float x, float y) {


        if (x == 0 && y == 0) {
            inScroll = false;
            gabrielClientActivity.setXY(0, 0);
            return true;
        }


        if (Math.abs(x) > Math.abs(y)) {
            gabrielClientActivity.setXY(x, 0);
        } else {
            gabrielClientActivity.setXY(0, y);
        }
        inScroll = true;

//        if (event1.getAction() == MotionEvent.ACTION_UP || event2.getAction() == MotionEvent.ACTION_UP ) {
//            // The pointer has gone up, ending the gesture
//            gabrielClientActivity.setXY(0, 0);
//        }
//
//        if (event1.getAction() == MotionEvent.ACTION_CANCEL || event2.getAction() == MotionEvent.ACTION_CANCEL) {
//            // The pointer has gone up, ending the gesture
//            gabrielClientActivity.setXY(0, 0);
//        }

        return true;
    }

    @Override
    public void onShowPress(MotionEvent event) {
    }

    @Override
    public boolean onSingleTapUp(MotionEvent event) {
        return true;
    }



}