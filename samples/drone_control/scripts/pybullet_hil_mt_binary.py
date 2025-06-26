import os
import time
import argparse
import numpy as np
import threading
import pybullet as p
import serial
import struct
from datetime import datetime

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

UART_PORT = "/dev/ttyUSB1"
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 400
DEFAULT_CONTROL_FREQ_HZ = 400
DEFAULT_DURATION_SEC = 1000
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

TIME_SCALE = 0.2
BAUDRATE = 115200

# Binary packet header
HEADER = b'\xDE\xAD\xBE\xEF'

# number of state floats per drone
NSTATES = 12
NACTIONS = 4

class TinyMPCSerialInterface:
    def __init__(self, port, baudrate=BAUDRATE, timeout=0.1, debug=True):
        self.debug = debug
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        # flush initial banners
        time.sleep(2)
        self.ser.reset_input_buffer()
        if self.debug:
            print(f"[TinyMPC] Serial opened on {port}@{baudrate}")

    def send_states_all(self, obs_array):
        """
        Pack and send binary packet:
        [HEADER(4)] [num_drones(1)] [states drone0 (NSTATES floats)] ...
        """
        num = obs_array.shape[0]
        packet = bytearray(HEADER)
        packet += struct.pack('<B', num)
        for i in range(num):
            packet += struct.pack(f'<{NSTATES}f', *obs_array[i])
        self.ser.write(packet)
        if self.debug:
            print(f"[TinyMPC][TX] Sent {num} states")

    def read_action(self):
        """
        Non-blocking: read header + id + 4 floats
        Returns (drone_id, forces) or None
        """
        # peek for header
        header = self.ser.read(4)
        if len(header) < 4:
            return None
        if header != HEADER:
            # shift by one
            self.ser.read(1)
            return None
        # read id
        id_byte = self.ser.read(1)
        if len(id_byte) < 1:
            return None
        drone_id = id_byte[0]
        # read 4 floats
        data = self.ser.read(NACTIONS * 4)
        if len(data) < NACTIONS * 4:
            return None
        forces = struct.unpack('<4f', data)
        if self.debug:
            print(f"[TinyMPC][RX] id={drone_id} forces={forces}")
        return drone_id, np.array(forces)

    def close(self):
        self.ser.close()

# Utility functions

def quaternion_to_RodriguesParam(q, eps=1e-8):
    q1, q2, q3, q4 = q
    if abs(q4) < eps:
        return 0.0, 0.0, 0.0
    return q1/q4, q2/q4, q3/q4


def extract_state_inputs(observation):
    x, y, z = observation[0], observation[1], observation[2]
    vx, vy, vz = observation[10], observation[11], observation[12]
    r1, r2, r3 = quaternion_to_RodriguesParam(observation[3:7])
    dphi, dtheta, dpsi = observation[13], observation[14], observation[15]
    return [x, y, z, r1, r2, r3, vx, vy, vz, dphi, dtheta, dpsi]


def calculate_rpm(env, normalized_thrusts):
    max_thrust_N = 0.58 / 4
    actual_thrusts = (normalized_thrusts + 0.583) * max_thrust_N
    actual_thrusts_clipped = np.clip(actual_thrusts, 0, None)
    return np.sqrt(actual_thrusts_clipped / env.KF)


def create_target_marker():
    shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1,0,0,0.5])
    return p.createMultiBody(0, shape, -1, [0,0,0.5])


def run(**kwargs):
    num_drones = kwargs['num_drones']
    assert kwargs['gui'], "GUI must be enabled"

    env = CtrlAviary(
        drone_model=kwargs['drone'], num_drones=num_drones,
        initial_xyzs=np.array([[0.5*i,0,0.1] for i in range(num_drones)]),
        initial_rpys=np.zeros((num_drones,3)),
        physics=kwargs['physics'], neighbourhood_radius=10,
        pyb_freq=kwargs['simulation_freq_hz'],
        ctrl_freq=kwargs['control_freq_hz'], gui=kwargs['gui'],
        record=kwargs['record_video'], obstacles=kwargs['obstacles'],
        user_debug_gui=kwargs['user_debug_gui']
    )

    offsets = np.array([[0.5*i,0,0] for i in range(num_drones)])
    p_client = env.getPyBulletClient()
    target_sliders = [p.addUserDebugParameter(name, *rng, default)
                      for name, rng, default in [
                          ("Target X", [-2,2],0),
                          ("Target Y", [-2,2],0),
                          ("Target Z", [0,2],1)
                      ]]

    interface = TinyMPCSerialInterface(UART_PORT)
    action = np.zeros((num_drones,4))
    obs = np.zeros((num_drones,20))
    obs_lock = threading.Lock()

    def sim_loop():
        next_step = time.time() + env.CTRL_TIMESTEP / TIME_SCALE
        while True:
            # read GUI sliders
            tx, ty, tz = [p.readUserDebugParameter(s) for s in target_sliders]
            target = np.array([tx, ty, tz])

            obs_local, _, _, _, _ = env.step(action)
            # adjust target offsets
            with obs_lock:
                for i in range(num_drones):
                    obs_local[i][0:3] -= (target + offsets[i])
                obs[:num_drones] = obs_local[:num_drones]
            env.render()

            # send full-state packet every timestep
            with obs_lock:
                states = [extract_state_inputs(obs[i]) for i in range(num_drones)]
            interface.send_states_all(np.array(states, dtype=np.float32))

            time.sleep(max(0, next_step - time.time()))
            next_step += env.CTRL_TIMESTEP / TIME_SCALE

    def control_loop():
        while True:
            res = interface.read_action()
            if res is not None:
                recv_id, forces = res
                if 0 <= recv_id < num_drones:
                    rpm = calculate_rpm(env, forces)
                    action[recv_id] = rpm
            else:
                time.sleep(0.0005)

    def marker_loop(num_drones, offsets):
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1,0,0,0.5])
        markers = [p.createMultiBody(0, target_visual, -1, [0,0,1])
                   for _ in range(num_drones)]
        last = None
        while True:
            tx, ty, tz = [p.readUserDebugParameter(s) for s in target_sliders]
            cur = (tx,ty,tz)
            if cur != last:
                last = cur
                for i in range(num_drones):
                    pos = [tx+offsets[i][0], ty+offsets[i][1], tz+offsets[i][2]]
                    p.resetBasePositionAndOrientation(markers[i], pos, [0,0,0,1])
            time.sleep(0.05)

    # threads
    threading.Thread(target=sim_loop, daemon=True).start()
    threading.Thread(target=control_loop, daemon=True).start()
    # threading.Thread(target=marker_loop, args=(num_drones, offsets), daemon=True).start()
    # run until duration
    time.sleep(kwargs['duration_sec'])
    interface.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone', default=DEFAULT_DRONES, type=DroneModel, choices=DroneModel)
    parser.add_argument('--num_drones', default=DEFAULT_NUM_DRONES, type=int)
    parser.add_argument('--physics', default=DEFAULT_PHYSICS, type=Physics, choices=Physics)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool)
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VISION, type=str2bool)
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool)
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool)
    parser.add_argument('--obstacles', default=DEFAULT_OBSTACLES, type=str2bool)
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int)
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int)
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int)
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str)
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool)
    args = parser.parse_args()
    run(**vars(args))
