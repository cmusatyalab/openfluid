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

import java.util.function.Consumer;
import edu.cmu.cs.openfluid.GabrielClientActivity;
import edu.cmu.cs.gabriel.client.results.ErrorType;
import edu.cmu.cs.openfluid.R;

public class ErrorConsumer implements Consumer<ErrorType> {
    private final GabrielClientActivity gabrielClientActivity;
    private boolean shownError;

    public ErrorConsumer(GabrielClientActivity gabrielClientActivity) {
        this.gabrielClientActivity = gabrielClientActivity;
        this.shownError = false;
    }

    @Override
    public void accept(ErrorType errorType) {
        int stringId;
        switch (errorType) {
            case SERVER_ERROR:
                stringId = R.string.server_error;
                break;
            case SERVER_DISCONNECTED:
                stringId = R.string.server_disconnected;
                break;
            case COULD_NOT_CONNECT:
                stringId = R.string.could_not_connect;
                break;
            default:
                stringId = R.string.unspecified_error;
        }
        this.showErrorMessage(stringId);
    }

    public void showErrorMessage(int stringId) {
        if (this.shownError) {
            return;
        }

        this.shownError = true;
        this.gabrielClientActivity.showNetworkErrorMessage(
                this.gabrielClientActivity.getResources().getString(stringId));
    }
}
