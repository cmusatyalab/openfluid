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

import android.app.AlertDialog;
import android.os.Build;
import android.util.Pair;
import android.view.HapticFeedbackConstants;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import androidx.annotation.RequiresApi;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class CamMode extends AbstractModeManager {

    public CamMode(GabrielClientActivity gabrielClientActivity, Map<ViewID, View> views) {
        super(gabrielClientActivity, views, new EnumMap<ViewID, Integer>(ViewID.class) {
            {
                put(ViewID.ARROW_UP, View.VISIBLE);
                put(ViewID.ARROW_DOWN, View.VISIBLE);
                put(ViewID.ARROW_LEFT, View.VISIBLE);
                put(ViewID.ARROW_RIGHT, View.VISIBLE);
                put(ViewID.FULL_SCREEN, View.VISIBLE);
                put(ViewID.CAM_CONTROL, View.VISIBLE);
                put(ViewID.AR_VIEW, View.VISIBLE);
                put(ViewID.ALIGN_CENTER, View.VISIBLE);
                put(ViewID.MENU, View.GONE);
                put(ViewID.SCENE_LIST, View.INVISIBLE);
                put(ViewID.RESET, View.INVISIBLE);
                put(ViewID.PLAY_PAUSE, View.INVISIBLE);
                put(ViewID.PARTICLE, View.INVISIBLE);
                put(ViewID.AUTO_PLAY, View.INVISIBLE);
                put(ViewID.ROTATE, View.INVISIBLE);
                put(ViewID.INFO, View.INVISIBLE);
                put(ViewID.HELP, View.VISIBLE);
                put(ViewID.MAIN, View.VISIBLE);
            }});
    }

    @Override
    public void init() {
        super.init();
        ((ImageView)this.views.get(ViewID.CAM_CONTROL)).setImageResource(R.drawable.baseline_keyboard_arrow_left_24);

    }

    @RequiresApi(api = Build.VERSION_CODES.O_MR1)
    @Override
    protected View.OnTouchListener getOnTouchListener(ViewID key) {

        if (key == ViewID.MAIN) {
            return new SceneScaleGestures(clientActivity, clientActivity);
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

        switch(key) {
            case FULL_SCREEN:

                return (view -> {
                    ((ImageView)this.views.get(ViewID.CAM_CONTROL)).setImageResource(R.drawable.baseline_control_camera_24);
                    clientActivity.switchMode(GabrielClientActivity.AppMode.FULLSCREEN);
                });
            case CAM_CONTROL:

                return (view -> {
                    ((ImageView)this.views.get(ViewID.CAM_CONTROL)).setImageResource(R.drawable.baseline_control_camera_24);
                    clientActivity.switchMode(GabrielClientActivity.AppMode.MAIN);
                });

            case ALIGN_CENTER:
                return (view -> {
                    clientActivity.setAlignCenter();
                });

            case AR_VIEW:
                return (view -> {
                    clientActivity.setARView();
                });

            case HELP:
                List<Pair<Integer, String>> items = new ArrayList<>();
                items.add(new Pair<>(R.drawable.pinch_fill0_wght400_grad0_opsz48,
                        clientActivity.getString(R.string.help_cma_pinch_zoom)));
                items.add(new Pair<>(R.drawable.drag_pan_fill0_wght400_grad0_opsz48__1_,
                        clientActivity.getString(R.string.help_cma_drag_pan)));
                items.add(new Pair<>(R.drawable._60_fill0_wght400_grad0_opsz48,
                        clientActivity.getString(R.string.help_cam_360)));
                items.add(new Pair<>(R.drawable._60_fill0_wght400_grad0_opsz48,
                        clientActivity.getString(R.string.help_cam_360_2)));
                items.add(new Pair<>(R.drawable.baseline_horizontal_distribute_24,
                        clientActivity.getString(R.string.help_cam_center)));
                items.add(new Pair<>(R.drawable.baseline_view_in_ar_24,
                        clientActivity.getString(R.string.help_cam_ar)));
                items.add(new Pair<>(R.drawable.baseline_keyboard_arrow_left_24,
                        clientActivity.getString(R.string.help_cam_back)));

                CustomListAdapter adapter = new CustomListAdapter(clientActivity, R.layout.my_help_list, items);
                AlertDialog.Builder builder = new AlertDialog.Builder(clientActivity);
                builder.setTitle("Camera Controls")
                        .setAdapter(adapter, (dialog, which) -> {});
                AlertDialog dialog = builder.create();

                return (view -> {
                    dialog.show();
                });
        }

        return null;
    }



}
