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

import android.app.Application;
import com.google.protobuf.ByteString;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.cmu.cs.gabriel.Const;
import edu.cmu.cs.openfluid.GabrielClientActivity;
import edu.cmu.cs.gabriel.client.comm.ServerComm;
import edu.cmu.cs.gabriel.protocol.Protos.InputFrame;
import edu.cmu.cs.gabriel.protocol.Protos.ResultWrapper;

public class OpenfluidComm {
    private final ServerComm serverComm;
    private final ErrorConsumer onDisconnect;

    public static OpenfluidComm createOpenfluidComm(
            String endpoint, int port, GabrielClientActivity gabrielClientActivity,
            Consumer<ByteString> imageView, String tokenLimit) {

        Consumer<ResultWrapper> consumer = new ResultConsumer(imageView, gabrielClientActivity);
        ErrorConsumer onDisconnect = new ErrorConsumer(gabrielClientActivity);
        ServerComm serverComm;
        Application application = gabrielClientActivity.getApplication();

        if (tokenLimit.equals("None")) {
            serverComm = ServerComm.createServerComm(
                    consumer, endpoint, port, application, onDisconnect);
        } else {
            serverComm = ServerComm.createServerComm(
                    consumer, endpoint, port, application, onDisconnect,
                    Integer.parseInt(tokenLimit));
        }

        return new OpenfluidComm(serverComm, onDisconnect);
    }

    OpenfluidComm(ServerComm serverComm, ErrorConsumer onDisconnect) {
        this.serverComm = serverComm;
        this.onDisconnect = onDisconnect;
    }

    public void sendSupplier(Supplier<InputFrame> supplier) {
        if (!this.serverComm.isRunning()) {
            return;
        }

        this.serverComm.sendSupplier(supplier, Const.SOURCE_NAME, /* wait */ false);
    }

    public void stop() {
        this.serverComm.stop();
    }
}
