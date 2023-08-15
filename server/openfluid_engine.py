# OpenFluid
#   - Real-time Fluid Simulation
#
#   Authors: Jinho Yi <jinhoy@andrew.cmu.edu>
#           Zhuo Chen <zhuoc@cs.cmu.edu>
#           Shilpa George <shilpag@andrew.cmu.edu>
#           Thomas Eiszler <teiszler@andrew.cmu.edu>
#           Padmanabhan Pillai <padmanabhan.s.pillai@intel.com>
#           Roger Iyengar <iyengar@cmu.edu>
#           Meng Cao <mcao@andrew.cmu.edu>
#
#   Copyright (C) 2011-2020 Carnegie Mellon University
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#

from gabriel_protocol import gabriel_pb2
from gabriel_server import cognitive_engine
from threading import Event, RLock, Thread
import logging
import openfluid_pb2
import os
import signal
import subprocess
import sys
import time
import zmq

logger = logging.getLogger(__name__)

class OpenfluidEngine(cognitive_engine.Engine):
    SOURCE_NAME = "openfluid"
    REQUEST_TIMEOUT = 2000
    
    instances = []
    scene_list = {}

    def sim_init_signal(self, signum, frame):
        self.signal_event.set()


    def __init__(self, zmq_port=5559, timeout=30, vsync=0):
        OpenfluidEngine.instances.append(self)
        logger.info("Initializing OpenfluidEngine")
        
        self.lock = RLock()
        signal.signal(signal.SIGUSR1, self.sim_init_signal)
        self.signal_event = Event()

        # Initialize ZeroMQ Context and Socket addrr
        self.zmq_context = zmq.Context()
        self.zmq_frame_addr = str(zmq_port)
        self.zmq_imu_addr = str(zmq_port + 1)

        # Initialize Screen Resolution
        self.screen_w = 480
        self.screen_h = 640
        self.screen_ratio = self.screen_h / self.screen_w

        # Pointer to the Physics Simulation Engine Process
        self.phys_simulator = None

        # Scene List retreived
        self.sendStyle = True

        # Initialize simulation engine and Client activity monitor
        self.activity_monitor = None
        self.fps_limit = 120 + vsync
        self.start_sim()

        self.get_scenes()
        self.monitor_stop = False
        self.client_event = Event()      
        self.activity_monitor = Thread(target = self.client_activity_monitor, args=(timeout, ))
        self.activity_monitor.start()

        # For Performance Measurement
        self.latency_token = 0
        self.server_fps = 0
        

        logger.info("\nFINISHED INITIALISATION")
        

    def release(self):
        logger.info("Terminating OpenfluidEngine")
        
        self.monitor_stop = True
        if self.activity_monitor != None:
            self.client_event.set()
            self.activity_monitor.join()
        logger.info("time Monitor killed")
        
        self.terminate_sim()
        

    # Turn off the simulation engine when no client is detected. 
    def client_activity_monitor(self, timeout=30):
        while not self.monitor_stop:
            if self.client_event.wait(timeout = timeout):
                self.client_event.clear()
            else:
                logger.info("Client inactive, terminating the simulation engine...")
                with self.lock:
                    self.terminate_sim()
                self.client_event.wait()
                self.client_event.clear()
    

    def close_socket(self):
        try:
            self.frame_socket.setsockopt(zmq.LINGER, 0)
            self.frame_socket.close()
        except:
            logger.info("Socket Already Closed")

        try:
            self.imu_socket.setsockopt(zmq.LINGER, 0)
            self.imu_socket.close()
        except:
            logger.info("Socket Already Closed")

    def setup_socket(self):
        self.frame_socket = self.zmq_context.socket( zmq.REQ )
        self.frame_socket.connect("ipc:///tmp/openfluid" + self.zmq_frame_addr)
        self.frame_socket.RCVTIMEO = self.REQUEST_TIMEOUT

        self.imu_socket = self.zmq_context.socket( zmq.PUSH )        
        self.imu_socket.setsockopt(zmq.SNDHWM , 1)
        self.imu_socket.connect("ipc:///tmp/openfluid" + self.zmq_imu_addr)


    def terminate_sim(self):
        with self.lock:
            if self.phys_simulator != None:
                os.killpg(os.getpgid(self.phys_simulator.pid), signal.SIGTERM)
                self.phys_simulator.kill()
                while self.phys_simulator.poll() is None:
                    time.sleep(0.1)
            
                self.phys_simulator = None

                logger.info("Flex Simulator Terminated")


    def start_sim(self):
        prog_path = os.path.join(os.path.abspath(sys.path[0]), 'Flex/bin/linux64/NvFlexDemoReleaseCUDA_x64')
        with self.lock:
            ARGS = [prog_path, 
                    f'-vsync={self.fps_limit % 10}',
                    f'-zmqport=' + self.zmq_frame_addr,
                    f'-windowed={self.screen_w }x{self.screen_h}',
                    f'-pid={os.getpid()}',
                    f'-fpslimit={int(self.fps_limit / 10) * 10}']
            logger.info("New Flex Simulator starting...")
            self.phys_simulator = subprocess.Popen(ARGS, start_new_session=True)

        self.signal_event.wait()
        self.setup_socket()


    def reset_simulator(self):
        self.terminate_sim()
        self.close_socket()
        self.start_sim()

    def get_scenes(self):
        logger.info("\nUpdating scenes, sending request...")

        while True:
            try:
                self.frame_socket.send_string("1")
                reply = self.frame_socket.recv()
                break
            except zmq.Again:
                logger.info("No response from the server, resending Scene Request...")
                # self.close_socket()   # Re-initiating Sockets would be suficient
                # self.setup_socket()
                self.reset_simulator() # Just in case that process got killed

        extras = openfluid_pb2.Extras()
        extras.ParseFromString(reply)
        OpenfluidEngine.scene_list = dict()
        for key in extras.style_list:
            OpenfluidEngine.scene_list[key] = extras.style_list[key]
        logger.info("Got Scene reply. Scene-lists Updated")
    

    def get_frame(self):
        # Request of new rendered frame to the Simulation Engine
        while True:
            try:
                self.frame_socket.send_string("0")
                reply = self.frame_socket.recv()
                break
            except zmq.Again:
                logger.info("No response from the Simulation Engine, restarting it")
                # self.close_socket()   # Re-initiating Sockets would be suficient
                # self.setup_socket()
                self.reset_simulator() # Just in case that process got killed
            
        input_frame = gabriel_pb2.InputFrame()
        input_frame.ParseFromString(reply)
        
        try:
            extras = cognitive_engine.unpack_extras(openfluid_pb2.Extras, input_frame)
            self.latency_token = extras.latency_token
            self.server_fps = extras.fps
        except:
            self.latency_token = 0

        return input_frame.payloads[0]
    

    def send_imu(self, extras):
        #Push IMU_data to the Simulation Engine
        self.imu_socket.send(extras.SerializeToString())
        return


    def handle(self, input_frame):
        self.client_event.set()

        # Check Input
        if input_frame.payload_type != gabriel_pb2.PayloadType.IMAGE:
            status = gabriel_pb2.ResultWrapper.Status.WRONG_INPUT_FORMAT
            return cognitive_engine.create_result_wrapper(status)
        
        # Retrieve data sent by the client
        extras = cognitive_engine.unpack_extras(openfluid_pb2.Extras, input_frame)
        
        # Check if Scene info needs to be updated
        if extras.setting_value.scene == -1:
            self.sendStyle = True

        with self.lock:
            if (self.screen_ratio != extras.screen_value.ratio) or (self.screen_w != extras.screen_value.resolution) or (extras.fps != self.fps_limit):
                self.screen_ratio = extras.screen_value.ratio
                self.screen_w = extras.screen_value.resolution
                self.screen_h = int(self.screen_ratio * self.screen_w + 0.5)
                self.fps_limit = extras.fps
                logger.info(f'-windowed={self.screen_w }x{self.screen_h} reset')
                self.reset_simulator()

        with self.lock:
            if self.phys_simulator == None:
                logger.info(f'-windowed={self.screen_w }x{self.screen_h} new')
                self.start_sim()
        
        # send imu data/get new rendered frame from/to the Physics simulation Engine
        self.send_imu(extras)
        img_data = self.process_image(None)

        # Serialize the result (protobuf)
        result = gabriel_pb2.ResultWrapper.Result()
        result.payload_type = gabriel_pb2.PayloadType.IMAGE
        result.payload = img_data

        extras = openfluid_pb2.Extras()
        if self.sendStyle:
            for k, v in self.scene_list.items():
                extras.style_list[k] = v
            self.sendStyle = False

        
        extras.latency_token = self.latency_token
        extras.fps = self.server_fps
            
        status = gabriel_pb2.ResultWrapper.Status.SUCCESS

        result_wrapper = cognitive_engine.create_result_wrapper(status)
        result_wrapper.results.append(result)
        result_wrapper.extras.Pack(extras)

        return result_wrapper


    def process_image(self, image):
        post_inference = self.inference(image)
        return post_inference


    def inference(self, image):
        return self.get_frame()