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

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.renderscript.RenderScript;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.MediaController;
import android.widget.TextView;
import android.widget.Toast;

import android.view.Choreographer;
import androidx.appcompat.app.AppCompatActivity;


import com.google.protobuf.Any;
import com.google.protobuf.ByteString;

import java.net.URI;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

import edu.cmu.cs.gabriel.Const;
import edu.cmu.cs.gabriel.camera.ImageViewUpdater;
import edu.cmu.cs.gabriel.network.OpenfluidComm;
import edu.cmu.cs.gabriel.protocol.Protos.InputFrame;
import edu.cmu.cs.gabriel.protocol.Protos.PayloadType;
import edu.cmu.cs.openfluid.Protos.Extras;

public class GabrielClientActivity extends AppCompatActivity implements SensorEventListener {

    public enum AppMode {
        MAIN, MENU, CAM, FULLSCREEN
    }
    private static boolean running = false;
    private static final String LOG_TAG = "GabrielClientActivity";

    private static final int LATENCY_TIMEOUT = 3000; //ms

    // major components for streaming sensor data and receiving information
    String serverIP = null;
    private String styleType = "-1"; //

    private OpenfluidComm openfluidComm;

    private MediaController mediaController = null;
    private float pxToDp;
    private int mScreenHeight = 640;
    private int mScreenWidth = 480;
    private float mScreenRatio = 1.0f;
    private int mResolution = Const.IMAGE_RES; // replace with Const later

    private int latencyToken = 0;

    // views
    private ImageView imgView;
    private ImageButton buttonLeft;
    private ImageButton buttonRight;
    private ImageButton buttonUp;
    private ImageButton buttonDown;

    private final Map<ViewID, View> views = new EnumMap<ViewID, View>(ViewID.class);
    private final Map<AppMode, AbstractModeManager> modeList = new EnumMap<AppMode, AbstractModeManager>(AppMode.class);

    private AppMode currMode = AppMode.MAIN;
    private Handler frameHandler;
    private Handler autoplayHandler;
    private Handler fpsHandler;
    private TextView fpsLabel;

    private TextView loadingLabel;

    private int framesProcessed = 0;
    private int serverFPS = 0;

    private final List<String> styleDescriptions = new ArrayList<>();
    private final List<String> styleIds = new ArrayList<>();

    private ArrayAdapter<String> sceneAdapter;

    public static class Pair implements Comparable<Pair> {
        String key;
        String value;

        public Pair(String key, String value) {
            this.key = key;
            this.value = value;
        }

        public String getKey() {
            return key;
        }

        public String getValue() {
            return value;
        }

        @Override
        public int compareTo(Pair other) {
            return Integer.parseInt(this.getKey()) - Integer.parseInt(other.getKey());
        }
    }

    public void sortPairedArray(List<String> keyList, List<String> valueList) {
        ArrayList<Pair> pairs = new ArrayList<>();
        for (int i = 0; i < keyList.size(); i++) {
            pairs.add(new Pair(keyList.get(i), valueList.get(i)));
        }

        Collections.sort(pairs);

        for (int i = 0; i < pairs.size(); i++) {
            Pair pair = pairs.get(i);
            keyList.set(i, pair.getKey());
            valueList.set(i, pair.getValue());
        }
    }

    public ArrayAdapter<String> getSceneAdapter() {
        return sceneAdapter;
    }

    public List<String> getSceneIDs() {
        return styleIds;
    }

    public List<String> getSceneDescriptions() {
        return styleDescriptions;
    }

    public void setSceneType(String sceneID) {
        styleType = sceneID;
    }

    public void addStyles(Set<Map.Entry<String, String>> entrySet) {
        for (Map.Entry<String, String> entry : entrySet) {
            Log.v(LOG_TAG, "style: " + entry.getKey() + ", desc: " + entry.getValue());
            styleDescriptions.add(entry.getValue());
            styleIds.add(entry.getKey());
        }

        // Sort the list of Pairs
        sortPairedArray(styleIds, styleDescriptions);

        this.styleType = styleIds.get(0);
    }

    public int getLatencyToken() {
        return latencyToken;
    }

    // SensorListener
    private SensorManager sensorManager;
    private Sensor mSensor;
    private float imu_x = 0;
    private float imu_y = 0;
    private float imu_z = 0;

    private float sceneScaleFactor = 1.0f;
    private float sceneX = 0.0f;
    private float sceneY= 0.0f;

    private boolean alignCenter = false;
    private boolean arView = false;
    private boolean reset = false;
    private boolean pause = false;
    private boolean particle = false;
    private boolean autoPlay = false;
    private boolean landscapeMode = false;
    private boolean info = false;

    public void setAlignCenter(){
        alignCenter = true;
    }

    public void setARView(){
        arView = true;
    }

    public void setReset(){
        reset = true;
    }

    public boolean getPause() {
        return pause;
    }

    public void setPause(boolean b){
        pause = !pause;
    }

    public boolean getParticle() {
        return particle;
    }

    public void setParticle(boolean b){
        particle = !particle;
    }

    public boolean getAutoPlay() {
        return autoPlay;
    }

    public void setAutoPlay(boolean b){
        autoPlay = !autoPlay;

        if (autoPlay) {
            setIMUSensor(false);
            autoplayHandler.postDelayed(gForceIterator, 100 );
            Toast.makeText(this, "Autoplay On", Toast.LENGTH_SHORT).show();

        } else {
            if(autoplayHandler != null) {
                autoplayHandler.removeCallbacks(gForceIterator);
            }
            setIMUSensor(true);
            Toast.makeText(this, "Autoplay Off", Toast.LENGTH_SHORT).show();
        }
    }

    public boolean getLandscapeMode() {
        return landscapeMode;
    }
    public void setLandscapeMode(boolean b){
        landscapeMode = !landscapeMode;
        if (landscapeMode) {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            // Swap
            int tmp = mScreenHeight;
            mScreenHeight = mScreenWidth;
            mScreenWidth = tmp;

            mScreenRatio = (float)((double)mScreenHeight / (double)mScreenWidth);
            mResolution = (int) ((float) Const.IMAGE_RES / mScreenRatio + 0.5f);
        } else {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
            // Swap
            int tmp = mScreenHeight;
            mScreenHeight = mScreenWidth;
            mScreenWidth = tmp;

            mScreenRatio = (float)((double)mScreenHeight / (double)mScreenWidth);
            mResolution = Const.IMAGE_RES;
        }
    }

    public boolean getInfo() {
        return info;
    }
    public void setInfo(boolean b){
        info = !info;
        if (info) {
            latencyToken = 1;
            latencyStartTime = System.currentTimeMillis();
            fpsLabel.setVisibility(View.VISIBLE);
        } else {
            fpsLabel.setVisibility(View.INVISIBLE);
            latencyToken = 0;
        }
    }

    public void setScaleFactor(float factor){
        sceneScaleFactor = factor;
    }

    public void setXY(float x, float y){
        sceneX += x;
        sceneY += y;
    }

    public void updateScreenSize() {
        DisplayMetrics metrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(metrics);
        pxToDp = 160.0f/metrics.densityDpi;
        mScreenHeight = metrics.heightPixels;
        mScreenWidth = metrics.widthPixels;
        mScreenRatio = (float)((double)mScreenHeight/ (double)mScreenWidth );
        mResolution = Const.IMAGE_RES;
    }


    public void enterFullscreen() {
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                        // Hide the nav bar and status bar
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_FULLSCREEN);
    }

    public void exitFullscreen() {
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_VISIBLE);
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().clearFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
    }

    public void switchMode(AppMode mode) {
        currMode = mode;
        modeList.get(mode).init();
    }

    public void setLoadingLable(boolean b) {
        if (b) {
            loadingLabel.setVisibility(View.VISIBLE);
        } else {
            loadingLabel.setVisibility(View.INVISIBLE);
        }
    }

    private boolean doubleTouch = false;
    public boolean isDoubleTouch() {
        return doubleTouch;
    }
    public void setDoubleTouch(boolean b) {
        doubleTouch = b;
    }

    @Override
    public final void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
    }

    @Override
    public final void onSensorChanged(SensorEvent event) {
        // The light sensor returns a single value.
        // Many sensors return 3 values, one for each axis.

        if (landscapeMode) {
            imu_x = -event.values[1];
            imu_y = event.values[0];
            imu_z = event.values[2];
        } else {
            imu_x = event.values[0];
            imu_y = event.values[1];
            imu_z = event.values[2];
        }
    }

    // local execution
    private long latencyStartTime;
    private long latencyMeasure = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.v(LOG_TAG, "++onCreate");
        super.onCreate(savedInstanceState);
        Const.STYLES_RETRIEVED = false;
        Const.ITERATION_STARTED = false;

        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_SHOW_WHEN_LOCKED
                + WindowManager.LayoutParams.FLAG_TURN_SCREEN_ON
                + WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        
        fpsLabel = findViewById(R.id.fpsLabel);
        loadingLabel = findViewById(R.id.loadingMsg);

        buttonLeft = findViewById(R.id.button_left);
        buttonRight = findViewById(R.id.button_right);
        buttonUp = findViewById(R.id.button_up);
        buttonDown = findViewById(R.id.button_down);
        imgView = findViewById(R.id.guidance_image);

        views.put(ViewID.ARROW_UP, buttonUp);
        views.put(ViewID.ARROW_DOWN, buttonDown);
        views.put(ViewID.ARROW_LEFT, buttonLeft);
        views.put(ViewID.ARROW_RIGHT, buttonRight);
        views.put(ViewID.FULL_SCREEN, (View) findViewById(R.id.imgFullScreen));
        views.put(ViewID.CAM_CONTROL, (View) findViewById(R.id.imgCamControl));
        views.put(ViewID.AR_VIEW, (View) findViewById(R.id.imgARView));
        views.put(ViewID.ALIGN_CENTER, (View) findViewById(R.id.imgAlignCenter));
        views.put(ViewID.MENU, (View) findViewById(R.id.imgMenu));
        views.put(ViewID.SCENE_LIST, (View) findViewById(R.id.imgSceneList));
        views.put(ViewID.RESET, (View) findViewById(R.id.imgReset));
        views.put(ViewID.PLAY_PAUSE, (View) findViewById(R.id.imgPlayPause));
        views.put(ViewID.PARTICLE, (View) findViewById(R.id.imgParticle));
        views.put(ViewID.AUTO_PLAY, (View) findViewById(R.id.imgAutoPlay));
        views.put(ViewID.ROTATE, (View) findViewById(R.id.imgRotate));
        views.put(ViewID.INFO, (View) findViewById(R.id.imgInfo));
        views.put(ViewID.HELP, (View) findViewById(R.id.imgHelp));
        views.put(ViewID.MAIN, imgView);

        modeList.put(AppMode.MAIN, new MainMode(this, views));
        modeList.put(AppMode.CAM, new CamMode(this, views));
        modeList.put(AppMode.MENU, new MenuMode(this, views));
        modeList.put(AppMode.FULLSCREEN, new FullScreenMode(this, views));
        switchMode(AppMode.MAIN);

        // Sensor Registration
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> gravSensors = sensorManager.getSensorList(Sensor.TYPE_ACCELEROMETER);
        mSensor = gravSensors.get(0);

        sceneAdapter = new ArrayAdapter<>(
                this, R.layout.mylist, styleDescriptions);

        autoplayHandler = new Handler();
        if (autoPlay)
            autoplayHandler.postDelayed(gForceIterator, 1000);

        fpsLabel.setVisibility(info ? View.VISIBLE : View.INVISIBLE);
        fpsHandler = new Handler();
        fpsHandler.postDelayed(fpsCalculator, 1000);

        updateScreenSize();
        frameHandler = new Handler();
    }

    private final Runnable frameIterator = new Runnable() {
        @Override
        public void run() {
            Choreographer.getInstance().postFrameCallback(frameCallback);
        }
    };

    private final Choreographer.FrameCallback frameCallback = new Choreographer.FrameCallback() {
        @Override
        public void doFrame(long frameTimeNanos) {
            // This code will run every time a new frame is drawn
//            framesProcessed++;
            sendIMUCloudlet();
            // Repost frame callback for the next frame
            if (running) {
                Choreographer.getInstance().postFrameCallback(this);
            }
        }
    };

    public void addFrameProcessed() {
        framesProcessed++;
        // Log.e("FRAME!!!!", "Frame is  " + framesProcessed);
    }

    private int avgServerFPS = 0;
    private int fpsCount;

    public void updateServerFPS(int fps) {
        fpsCount++;
        serverFPS += fps;
        avgServerFPS = (int) ((float) serverFPS / (float) fpsCount + 0.5);

        if (fpsCount >= 120){
            fpsCount = 0;
            serverFPS = 0;
        }
    }

    private double avgLatency = 0.0;
    private long latencyCount = 0;

    public void updateLatency() {
        latencyCount++;
        long endTime = System.currentTimeMillis();
        latencyMeasure += (endTime - latencyStartTime);
        avgLatency = (double) latencyMeasure / (double) latencyCount;
        latencyToken += 1;
        if (latencyToken == Integer.MAX_VALUE) {
            latencyToken = 1;
        }
        latencyStartTime = endTime;

        if (latencyCount >= 100){
            latencyCount = 0;
            latencyMeasure = 0;
        }
    }

    private final Runnable fpsCalculator = new Runnable() {
        @SuppressLint("DefaultLocale")
        @Override
        public void run() {
            String msg = "FPS-Gabrial: " + framesProcessed;
            msg += "\nFPS-Simulation: " + avgServerFPS;
            msg += " \nLatency: " + String.format("%.2f", avgLatency) + "ms";
            fpsLabel.setText( msg );

            // if ((System.currentTimeMillis() - latencyStartTime > LATENCY_TIMEOUT) && info) {
            //     latencyToken = true;
            //     latencyStartTime = System.currentTimeMillis();
            // }


            framesProcessed = 0;
            fpsHandler.postDelayed(this, 1000);
        }
    };

    private final int[] xSequence = new int[] {0, 1, 0,-1, 0, 0, 0, 0 ,0, 0,-1, 0, 1, 0,-1, 0, 0, 1, 0, 0, 0, 0};
    private final int[] ySequence = new int[] {1, 0,-1, 0, 1, 0,-1, 0, 0, 1, 0,-1, 0, 0, 0, 1, 0, 0,-1, 0, 0, 0};
    private final int[] zSequence = new int[] {0, 0, 0, 0, 0, 1, 0,-1, 1, 0, 0, 0, 0,-1 ,0, 0, 1, 0, 0,-1, 0 ,0};

    private int seqIdx = 0;
    private final Runnable gForceIterator = new Runnable() {
        @Override
        public void run() {
            if (autoPlay) {
                imu_x = xSequence[seqIdx] * 9.8f;
                imu_y = ySequence[seqIdx] * 9.8f;
                imu_z = zSequence[seqIdx] * 9.8f;
                seqIdx = (seqIdx + 1) % xSequence.length;
                autoplayHandler.postDelayed(this, Const.ITERATE_INTERVAL );
            }
        }
    };

    private boolean imuOn = false;
    void setIMUSensor(boolean on) {
        if (on) {
            if (!imuOn) {
                imuOn = true;
                sensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_NORMAL);
            }
        } else {
            if (imuOn) {
                imuOn = false;
                sensorManager.unregisterListener(this);
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.v(LOG_TAG, "++onResume");
        setIMUSensor(true);
        setLoadingLable(true);

        initOnce();
        Intent intent = getIntent();
        serverIP = intent.getStringExtra("SERVER_IP");
        initPerRun(serverIP);
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.v(LOG_TAG, "++onPause");
        setIMUSensor(false);

        if(frameHandler != null) {
            running = false;
            frameHandler.removeCallbacks(frameIterator);
        }

        if(autoplayHandler != null) {
            autoPlay = false;
            autoplayHandler.removeCallbacks(gForceIterator);
        }

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.v(LOG_TAG, "++onDestroy");

        if(frameHandler != null) {
            running = false;
            frameHandler.removeCallbacks(frameIterator);
        }

        if(autoplayHandler != null) {
            autoPlay = false;
            autoplayHandler.removeCallbacks(gForceIterator);
        }

        if (this.openfluidComm != null) {
            this.openfluidComm.stop();
            this.openfluidComm = null;
        }
    }

    /**
     * Does initialization for the entire application.
     */
    private void initOnce() {
        Log.v(LOG_TAG, "++initOnce");

        // Media controller
        if (mediaController == null) {
            mediaController = new MediaController(this);
        }
    }

    /**
     * Does initialization before each run (connecting to a specific server).
     */
    private void initPerRun(String serverIP) {
        Log.v(LOG_TAG, "++initPerRun");

        if (currMode == AppMode.FULLSCREEN)
            currMode = AppMode.MAIN;
        switchMode(currMode);

        if (serverIP == null) return;

        this.setupComm();

        running = true;
        frameHandler.post(frameIterator);

    }

    // Based on
    // https://github.com/protocolbuffers/protobuf/blob/2f6a7546e4539499bc08abc6900dc929782f5dcd/src/google/protobuf/compiler/java/java_message.cc#L1374
    private static Any pack(Extras extras) {
        return Any.newBuilder()
                .setTypeUrl("type.googleapis.com/openfluid.Extras")
                .setValue(extras.toByteString())
                .build();
    }

    int counter = 0;
    private void sendIMUCloudlet() {
        openfluidComm.sendSupplier(() -> {
            Extras.ScreenValue screenValue = Extras.ScreenValue.newBuilder()
                    .setResolution(mResolution)
                    .setRatio(mScreenRatio)
                    .build();

            Extras.IMUValue imuValue = Extras.IMUValue.newBuilder()
                    .setX(imu_x)
                    .setY(imu_y)
                    .setZ(imu_z)
                    .build();


            Extras.ArrowKey arrowKey = Extras.ArrowKey.newBuilder()
                    .setLeft(buttonLeft.isPressed())
                    .setRight(buttonRight.isPressed())
                    .setUp(buttonUp.isPressed())
                    .setDown(buttonDown.isPressed())
                    .build();

            Extras.Setting settingValues = Extras.Setting.newBuilder()
                    .setScene(Integer.parseInt(styleType))
                    .setAlignCenter(alignCenter)
                    .setArView(arView)
                    .setReset(reset)
                    .setPause(pause)
                    .setParticle(particle)
                    .setInfo(info)
                    .build();

            Extras.TouchInput touchValue = Extras.TouchInput.newBuilder()
                    .setX( sceneX * pxToDp)
                    .setY( sceneY * pxToDp)
                    .setScale(sceneScaleFactor)
                    .setDoubleTouch(doubleTouch)
                    .build();

            Extras extras = Extras.newBuilder().setSettingValue(settingValues)
                    .setScreenValue(screenValue)
                    .setImuValue(imuValue)
                    .setTouchValue(touchValue)
                    .setArrowKey(arrowKey)
                    .setLatencyToken(latencyToken)
                    .setFps((Const.VSYNC ? 1 : 0) + Const.CAPTURE_FPS)
                    .build();

            return InputFrame.newBuilder()
                    .setPayloadType(PayloadType.IMAGE)
                    .setExtras(GabrielClientActivity.pack(extras))
                    .build();
        });

        reset = false;
        arView = false;
        alignCenter = false;
        sceneX = 0;
        sceneY = 0;
        // latencyToken = false;
    }

    int getPort() {
        Log.d(LOG_TAG, this.serverIP);
        int port = URI.create(this.serverIP).getPort();
        if (port == -1) {
            return Const.PORT;
        }
        return port;
    }

    void setupComm() {
        int port = getPort();
        Consumer<ByteString> imageViewUpdater = new ImageViewUpdater(this.imgView);
        this.openfluidComm = OpenfluidComm.createOpenfluidComm(
                this.serverIP, port, this, imageViewUpdater, Const.TOKEN_LIMIT);
    }

    void setOpenfluidComm(OpenfluidComm openfluidComm) {
        this.openfluidComm = openfluidComm;
    }

    public void showNetworkErrorMessage(String message) {
        // suppress this error when screen recording as we have to temporarily leave this
        // activity causing a network disruption
        this.runOnUiThread(() -> {
            AlertDialog.Builder builder = new AlertDialog.Builder(
                    this, android.R.style.Theme_Material_Light_Dialog_Alert);
            builder.setMessage(message)
                    .setTitle(R.string.connection_error)
                    .setNegativeButton(R.string.back_button,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    GabrielClientActivity.this.finish();
                                }
                            }).setCancelable(false);
            AlertDialog dialog = builder.create();
            dialog.show();
        });
    }
}
