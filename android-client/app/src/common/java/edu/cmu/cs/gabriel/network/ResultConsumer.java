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

package edu.cmu.cs.gabriel.network;

import android.util.Log;

import com.google.protobuf.ByteString;
import com.google.protobuf.InvalidProtocolBufferException;

import java.util.TreeMap;
import java.util.function.Consumer;

import edu.cmu.cs.gabriel.Const;
import edu.cmu.cs.openfluid.GabrielClientActivity;
import edu.cmu.cs.gabriel.protocol.Protos.ResultWrapper;
import edu.cmu.cs.gabriel.protocol.Protos.PayloadType;
import edu.cmu.cs.openfluid.Protos.Extras;

public class ResultConsumer implements Consumer<ResultWrapper> {
    private static final String TAG = "ResultConsumer";

    private final Consumer<ByteString> imageViewUpdater;
    private final GabrielClientActivity gabrielClientActivity;

    public ResultConsumer(Consumer<ByteString> imageViewUpdater,
            GabrielClientActivity gabrielClientActivity) {
        this.imageViewUpdater = imageViewUpdater;
        this.gabrielClientActivity = gabrielClientActivity;
    }

    @Override
    public void accept(ResultWrapper resultWrapper) {
        if (resultWrapper.getResultsCount() != 1) {
            Log.e(TAG, "Got " + resultWrapper.getResultsCount() + " results in output.");
            return;
        }
        this.gabrielClientActivity.setLoadingLable(false);

        ResultWrapper.Result result = resultWrapper.getResults(0);
        try {
            Extras extras = Extras.parseFrom(resultWrapper.getExtras().getValue());

            if (!Const.STYLES_RETRIEVED && (extras.getStyleListCount() > 0)) {
                Const.STYLES_RETRIEVED = true;
                this.gabrielClientActivity.addStyles(new TreeMap<String, String>(extras.getStyleListMap()).entrySet());
            }

            int lt = extras.getLatencyToken();
            if (lt != 0 && lt == this.gabrielClientActivity.getLatencyToken()) {
                this.gabrielClientActivity.updateLatency();
            }

            this.gabrielClientActivity.updateServerFPS(extras.getFps());
        }  catch (InvalidProtocolBufferException e) {
            Log.e(TAG, "Protobuf Error", e);
        }

        if (result.getPayloadType() != PayloadType.IMAGE) {
            Log.e(TAG, "Got result of type " + result.getPayloadType().name());
            return;
        }

        this.imageViewUpdater.accept(result.getPayload());
        this.gabrielClientActivity.addFrameProcessed();
    }
}

