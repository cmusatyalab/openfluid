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

import android.util.Log;
import android.view.View;

import java.util.Map;

public abstract class AbstractModeManager {

    protected Map<ViewID, View> views;
    protected Map<ViewID, Integer> visibility;
    protected static GabrielClientActivity clientActivity;

    public AbstractModeManager(GabrielClientActivity gabrielClientActivity, Map<ViewID, View> views, Map<ViewID, Integer> visibility) {
        this.clientActivity = gabrielClientActivity;
        this.views = views;
        this.visibility = visibility;

        Log.v("ModeManager", "Visibility Size = " + this.visibility.size());
        Log.v("ModeManager", "ViewID Size = " + ViewID.SIZE.getValue());

        assert this.visibility.size() == ViewID.SIZE.getValue();
    }

    public void init() {
        for (ViewID key : visibility.keySet()) {
            Integer view_visibility = visibility.get(key);
            View view = views.get(key);
            view.setVisibility(view_visibility);

            if (view_visibility == View.VISIBLE) {
                view.setOnTouchListener(getOnTouchListener(key));
                view.setOnClickListener(getOnClickListener(key));
            } else {
                view.setOnTouchListener(null);
                view.setOnClickListener(null);
            }
        }
    }

    protected abstract View.OnTouchListener getOnTouchListener(ViewID key);

    protected abstract View.OnClickListener getOnClickListener(ViewID key);
}

