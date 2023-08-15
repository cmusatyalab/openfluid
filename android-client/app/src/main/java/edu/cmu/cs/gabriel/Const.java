// Copyright 2018 Carnegie Mellon University
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

package edu.cmu.cs.gabriel;
import android.content.SharedPreferences;

public class Const {
    public static boolean ITERATION_STARTED = false;
    public static int ITERATE_INTERVAL = 2000;
    public static boolean VSYNC = true;

    public static boolean STYLES_RETRIEVED = false;

    // image size and frame rate
    public static int CAPTURE_FPS = 120;

    // options: 240p, 360p, 480p, 720p, 1080p, 1440p, 2160p
    public static int IMAGE_RES =  480;
    public static int IMAGE_WIDTH = 320;
    public static int IMAGE_HEIGHT = 240;

    public static final int PORT = 9099;

    // server IP
    public static String SERVER_IP = "";  // Cloudlet

    // token size
    public static String TOKEN_LIMIT = "None";

    public static final String SOURCE_NAME = "openfluid";

    public static void loadPref(SharedPreferences sharedPreferences, String key) {
        Boolean b = null;
        Integer i = null;
        //update Const values so that new settings take effect
        switch(key) {
            case "general_vsync":
                b = sharedPreferences.getBoolean(key, true);
                Const.VSYNC = b;
                break;
            case "experimental_resolution":
                i = new Integer(sharedPreferences.getString(key, "480"));
                Const.IMAGE_RES = i;
                break;
            case "experimental_token_limit":
                Const.TOKEN_LIMIT = sharedPreferences.getString(key, "None");
                break;
            case "general_FPS":
                i = new Integer(sharedPreferences.getString(key, "120"));
                Const.CAPTURE_FPS = i;
                break;
            case "general_iterate_delay":
                i = new Integer(sharedPreferences.getString(key, "2"));
                Const.ITERATE_INTERVAL = i * 1000;
                break;

        }
    }
}
