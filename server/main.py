#!/usr/bin/env python3

from gabriel_server import local_engine
from openfluid_engine import OpenfluidEngine
from timing_engine import TimingEngine
import logging
import argparse
import sys

DEFAULT_PORT = 9099
DEFAULT_ZMQ_PORT = 5559
DEFAULT_NUM_TOKENS = 5
INPUT_QUEUE_MAXSIZE = 240
DEFAULT_TIMEOUT = 30

import signal 
import sys

logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-t", "--tokens", type=int, default=DEFAULT_NUM_TOKENS, help="number of tokens"
    )
    parser.add_argument(
        "--timing", action="store_true", help="Print timing information"
    )
    parser.add_argument(
        "-p", "--port", type=int, default=DEFAULT_PORT, help="Set port number"
    )
    parser.add_argument(
        "--zmqport", type=int, default=DEFAULT_ZMQ_PORT, help="Set port for zmq IPC"
    )
    parser.add_argument(
        "--timeout", type=int, default=DEFAULT_TIMEOUT, help="Set client inactive duration before the simulation goes to sleep mode (sec)"
    )
    parser.add_argument(
        "-v", "--vsync", type=int, default=0, help="Set client inactive duration before the simulation goes to sleep mode (sec)"
    )
    
    args = parser.parse_args()

    def engine_setup():
        if args.timing:
            engine = TimingEngine(args.zmqport, args.timeout, args.vsync)
        else:
            engine = OpenfluidEngine(args.zmqport, args.timeout, args.vsync)

        return engine

    local_engine.run(
        engine_setup,
        OpenfluidEngine.SOURCE_NAME,
        INPUT_QUEUE_MAXSIZE,
        args.port,
        args.tokens,
    )

def signal_handler(signal, frame):
    print()

    for obj in OpenfluidEngine.instances:
        obj.release()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
