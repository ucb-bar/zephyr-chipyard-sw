# hil_named_disturbance_dict.py

import os
import json
import time
import argparse
import numpy as np
import pybullet as p
import serial
import struct

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.utils import str2bool

# UART_PORT = "/dev/ttyUSB1"
UART_PORT = "/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0"
BAUDRATE = 115200
HEADER = b'\xDE\xAD\xBE\xEF'
NSTATES = 12
NACTIONS = 4
TIME_SCALE = 0.2
IDEAL = False

def quaternion_to_RodriguesParam(q, eps=1e-8):
    q1, q2, q3, q4 = q
    if abs(q4) < eps:
        return 0.0, 0.0, 0.0
    return q1/q4, q2/q4, q3/q4

def extract_state_inputs(obs):
    x, y, z = obs[0], obs[1], obs[2]
    vx, vy, vz = obs[10], obs[11], obs[12]
    r1, r2, r3 = quaternion_to_RodriguesParam(obs[3:7])
    dphi, dtheta, dpsi = obs[13], obs[14], obs[15]
    return [x, y, z, r1, r2, r3, vx, vy, vz, dphi, dtheta, dpsi]

def calculate_rpm(env, normalized_thrusts):
    max_thrust_N = 0.58 / 4
    actual = (normalized_thrusts + 0.583) * max_thrust_N
    return np.sqrt(np.clip(actual, 0, None) / env.KF)
    
# HAWK
# def calculate_rpm(env, normalized_thrusts):
#     max_thrust_N = 3 * 0.58 / 4
#     # max_thrust_N = 6.96 * 0.58 / 4
#     # max_thrust_N = 1720 / 4
#     # actual = (normalized_thrusts + 0.0625) * max_thrust_N
#     actual = (normalized_thrusts + 0.03) * max_thrust_N
#     return np.sqrt(np.clip(actual, 0, None) / env.KF)

class TinyMPCSerialInterface:
    def __init__(self, port, baudrate=BAUDRATE, timeout=0, debug=True):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self._last_rx_time = {}
        self.debug = debug
        if self.debug:
            print(f"[TinyMPC] Serial opened on {port}@{baudrate}")

    def send_states_all(self, obs_array):
        num = obs_array.shape[0]
        packet = bytearray(HEADER)
        packet += struct.pack('<B', num)
        for i in range(num):
            packet += struct.pack(f'<{NSTATES}f', *obs_array[i])
        self.ser.write(packet)
        if self.debug:
            print(f"[TinyMPC][TX] Sent {num} states")

    def read_action(self):
        total_len = len(HEADER) + 1 + NACTIONS*4 + 4
        if self.ser.in_waiting < total_len:
            return None
        packet = self.ser.read(total_len)
        if len(packet) < total_len or packet[:len(HEADER)] != HEADER:
            return None
        offset = len(HEADER)
        drone_id = packet[offset]
        offset += 1
        forces = struct.unpack('<4f', packet[offset:offset + NACTIONS*4])
        offset += NACTIONS*4
        ns = struct.unpack('<i', packet[offset:offset+4])[0]
        return drone_id, np.array(forces), ns

    def close(self):
        self.ser.close()

def run(**kwargs):
    with open(kwargs['traj_file'], 'r') as f:
        maneuvers = json.load(f)

    if kwargs['dist_file']:
        with open(kwargs['dist_file'], 'r') as f:
            disturbance_map = json.load(f)
    else:
        disturbance_map = {"none": []}

    output_folder = kwargs['output_folder']
    os.makedirs(output_folder, exist_ok=True)

    env = CtrlAviary(
        drone_model=kwargs['drone'],
        num_drones=1,
        initial_xyzs=np.array([[0.0,0.0,0.0]]),
        initial_rpys=np.zeros((1,3)),
        physics=kwargs['physics'],
        neighbourhood_radius=10,
        pyb_freq=kwargs['simulation_freq_hz'],
        ctrl_freq=kwargs['control_freq_hz'],
        gui=kwargs['gui'],
        record=kwargs['record_video'],
        obstacles=kwargs['obstacles'],
        user_debug_gui=kwargs['user_debug_gui']
    )

    p_client = env.getPyBulletClient()
    DRONE_ID = env.DRONE_IDS[0]
    interface = TinyMPCSerialInterface(UART_PORT)
    zeros_action = np.zeros((1,4), dtype=np.float32)

    for dist_name, disturbance_schedule in disturbance_map.items():
        disturbance_schedule = sorted(disturbance_schedule, key=lambda x: x[6])

        for man in maneuvers:
            name = man['name']
            traj = sorted(man['traj'], key=lambda x: x[3])
            t_final = traj[-1][3]
            n_steps = int(np.floor(t_final / env.CTRL_TIMESTEP)) + 1


            dtype = [
                ('state', 'f4', (NSTATES,)),
                ('rpm', 'f4', (NACTIONS,)),
                ('tx', 'f4'), ('ty', 'f4'), ('tz', 'f4'),
                ('new_action', '?'),
                ('wall_time', 'f8'), ('ns', 'i8'),
                ('dist_force', 'f4', (3,)),     # NEW
                ('dist_torque', 'f4', (3,)),    # NEW
            ]
            log = np.zeros(n_steps, dtype=dtype)

            # Send a zero state and read action for 100 iterations to reset the controller
            # for _ in range(100):
            #     interface.send_states_all(np.array([0,0,0,0,0,0,0,0,0,0,0,0], dtype=np.float32).reshape(1, -1))
            #     res = interface.read_action()

            x0, y0, z0, _ = traj[0]
            p.resetBasePositionAndOrientation(DRONE_ID, [x0,y0,z0], [0,0,0,1], physicsClientId=p_client)
            p.resetBaseVelocity(DRONE_ID, [0,0,0], [0,0,0], physicsClientId=p_client)

            print(f"=== Starting '{name}' + disturbance '{dist_name}' ===")

            step = 0
            idx_traj = 0
            idx_dist = 0
            current_force = [0.0, 0.0, 0.0]
            current_torque = [0.0, 0.0, 0.0]
            action = zeros_action.copy()
            obs = np.zeros((1,20), dtype=np.float32)
            next_t = time.time()
            interval = 1.0 / kwargs['control_freq_hz'] / TIME_SCALE

            while step < n_steps:
                sim_time = step * 1.0 / kwargs['control_freq_hz']
                while idx_traj+1 < len(traj) and traj[idx_traj+1][3] <= sim_time:
                    idx_traj += 1
                tx, ty, tz, _ = traj[idx_traj]

                res = interface.read_action()
                if res is not None:
                    _, forces, ns = res
                    action[0] = calculate_rpm(env, forces)
                    new_action = True
                else:
                    ns = -1
                    new_action = False

                while idx_dist < len(disturbance_schedule):
                    fx, fy, fz = disturbance_schedule[idx_dist][0:3]
                    tx_, ty_, tz_ = disturbance_schedule[idx_dist][3:6]
                    t_dist = disturbance_schedule[idx_dist][6]
                    if sim_time >= t_dist:
                        current_force = [fx, fy, fz]
                        current_torque = [tx_, ty_, tz_]
                        print(f"[Disturbance:{dist_name}] t={sim_time:.2f}s force={current_force} torque={current_torque}")
                        idx_dist += 1
                    else:
                        break

                pos, _ = p.getBasePositionAndOrientation(DRONE_ID)
                p.applyExternalForce(DRONE_ID, -1, current_force, pos, p.WORLD_FRAME, physicsClientId=p_client)
                p.applyExternalTorque(DRONE_ID, -1, current_torque, p.LINK_FRAME, physicsClientId=p_client)
                obs_local, _, _, _, _ = env.step(action)
                obs_local[0][0:3] -= np.array([tx,ty,tz])
                obs[:] = obs_local
                env.render()

                # track the *first* drone (ID = 0) at a fixed distance
                cam_dist  = 1.0    # meters behind/above the drone
                cam_yaw   = -45     # degrees around z-axis
                cam_pitch = -30    # degrees down from horizontal

                # get the drone's world position
                drone_pos, _ = p.getBasePositionAndOrientation(DRONE_ID,
                                                            physicsClientId=p_client)

                # reposition the debug camera
                p.resetDebugVisualizerCamera(cameraDistance=cam_dist,
                                            cameraYaw=cam_yaw,
                                            cameraPitch=cam_pitch,
                                            cameraTargetPosition=drone_pos,
                                            physicsClientId=p_client)

                state_in = np.array(extract_state_inputs(obs[0]), dtype=np.float32)
                log['state'][step] = state_in
                log['rpm'][step] = action[0]
                log['tx'][step] = tx
                log['ty'][step] = ty
                log['tz'][step] = tz
                log['new_action'][step] = new_action
                log['wall_time'][step] = time.time()
                log['ns'][step] = ns
                log['dist_force'][step] = current_force
                log['dist_torque'][step] = current_torque


                interface.send_states_all(state_in.reshape(1,-1))
                sleep = next_t + interval - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                next_t += interval
                step += 1

            filename = f"{name}__{dist_name}.npy"
            out_path = os.path.join(output_folder, filename)
            np.save(out_path, log)
            print(f"--> Saved log to {out_path}")

            # Send a zero state and read action for 100 iterations to reset the controller
            for _ in range(40):
                interface.send_states_all(np.array([0,0,0,0,0,0,0,0,0,0,0,0], dtype=np.float32).reshape(1, -1))
                res = interface.read_action()
                time.sleep(0.1)
            for _ in range(20):
                res = interface.read_action()
                time.sleep(0.1)
    interface.close()
    p.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--traj_file', required=True)
    parser.add_argument('--dist_file', default=None)
    parser.add_argument('--drone', default=DroneModel("cf2x"), type=DroneModel)
    parser.add_argument('--num_drones', default=1, type=int)
    parser.add_argument('--physics', default=Physics("pyb"), type=Physics)
    parser.add_argument('--gui', default=True, type=str2bool)
    parser.add_argument('--record_video', default=False, type=str2bool)
    parser.add_argument('--user_debug_gui', default=False, type=str2bool)
    parser.add_argument('--obstacles', default=True, type=str2bool)
    parser.add_argument('--simulation_freq_hz', default=800, type=int)
    parser.add_argument('--control_freq_hz', default=800, type=int)
    parser.add_argument('--output_folder', default='results')
    args = parser.parse_args()
    run(**vars(args))
