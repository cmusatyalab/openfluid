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

import android.os.Build;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import androidx.annotation.RequiresApi;

import java.util.EnumMap;
import java.util.Map;

public class FullScreenMode extends AbstractModeManager {

    public FullScreenMode(GabrielClientActivity gabrielClientActivity, Map<ViewID, View> views) {
        super(gabrielClientActivity, views, new EnumMap<ViewID, Integer>(ViewID.class) {
            {
                put(ViewID.ARROW_UP, View.INVISIBLE);
                put(ViewID.ARROW_DOWN, View.INVISIBLE);
                put(ViewID.ARROW_LEFT, View.INVISIBLE);
                put(ViewID.ARROW_RIGHT, View.INVISIBLE);
                put(ViewID.FULL_SCREEN, View.VISIBLE);
                put(ViewID.CAM_CONTROL, View.INVISIBLE);
                put(ViewID.AR_VIEW, View.INVISIBLE);
                put(ViewID.ALIGN_CENTER, View.INVISIBLE);
                put(ViewID.MENU, View.INVISIBLE);
                put(ViewID.SCENE_LIST, View.INVISIBLE);
                put(ViewID.RESET, View.INVISIBLE);
                put(ViewID.PLAY_PAUSE, View.INVISIBLE);
                put(ViewID.PARTICLE, View.INVISIBLE);
                put(ViewID.AUTO_PLAY, View.INVISIBLE);
                put(ViewID.ROTATE, View.INVISIBLE);
                put(ViewID.INFO, View.INVISIBLE);
                put(ViewID.HELP, View.INVISIBLE);
                put(ViewID.MAIN, View.VISIBLE);
            }});
    }

    @Override
    public void init() {
        super.init();
        ((ImageView) this.views.get(ViewID.FULL_SCREEN)).setImageResource(R.drawable.baseline_fullscreen_exit_24);
        clientActivity.enterFullscreen();
    }

    @RequiresApi(api = Build.VERSION_CODES.O_MR1)
    @Override
    protected View.OnTouchListener getOnTouchListener(ViewID key) {

        if (key == ViewID.MAIN) {
            return null;
        }

        return (view, event) -> {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    view.setPressed(true);
                    view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY, HapticFeedbackConstants.FLAG_IGNORE_GLOBAL_SETTING);
                    break;
                case MotionEvent.ACTION_UP:
                    view.setPressed(false);
                    view.performClick();
                    view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY_RELEASE, HapticFeedbackConstants.FLAG_IGNORE_GLOBAL_SETTING);
                    break;
            }
            return true;
        };
    }

    @Override
    protected View.OnClickListener getOnClickListener(ViewID key) {

        if (key == ViewID.FULL_SCREEN) {
            return (view -> {
                ((ImageView) this.views.get(ViewID.FULL_SCREEN)).setImageResource(R.drawable.baseline_fullscreen_24);
                clientActivity.switchMode(GabrielClientActivity.AppMode.MAIN);
                clientActivity.exitFullscreen();
            });
        }

        return null;
    }



}
