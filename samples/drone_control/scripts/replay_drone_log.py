#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

def replay_log(log_file,
               drone_model=DroneModel("cf2x"),
               physics=Physics("pyb"),
               gui=True,
               replay_freq=None,
               video_out=None):
    print(f"[DEBUG] Loading log: {log_file}")
    data = np.load(log_file, allow_pickle=False)
    # reconstruct absolute positions
    targ   = np.stack([data['tx'], data['ty'], data['tz']], axis=1)
    relpos = data['state'][:, :3]
    poses  = relpos + targ
    # extract Rodrigues params
    rodr  = data['state'][:, 3:6]  # r1,r2,r3 per frame
    times = data['wall_time'] - data['wall_time'][0]
    n     = poses.shape[0]
    print(f"[DEBUG] {n} frames; duration {times[-1]:.3f}s")

    # init environment
    print("[DEBUG] Initializing CtrlAviary...")
    env = CtrlAviary(
        drone_model=drone_model,
        num_drones=1,
        initial_xyzs=np.zeros((1,3)),
        initial_rpys=np.zeros((1,3)),
        physics=physics,
        neighbourhood_radius=10,
        pyb_freq=1, ctrl_freq=1,
        gui=gui, record=False,
        obstacles=False, user_debug_gui=False
    )
    print(f"[DEBUG] Aviary client={env.CLIENT}")
    reset_res = env.reset()
    obs = reset_res[0] if isinstance(reset_res, tuple) else reset_res
    print(f"[DEBUG] reset() done; obs shape={np.array(obs).shape}")

    pid      = env.getPyBulletClient()
    DRONE_ID = env.DRONE_IDS[0]
    print(f"[DEBUG] Drone ID={DRONE_ID}")

    # optional video logging
    log_id = None
    if video_out:
        fps = int(replay_freq) if replay_freq else 30
        print(f"[DEBUG] Recording '{video_out}' at {fps} FPS")
        log_id = p.startStateLogging(
            p.STATE_LOGGING_VIDEO_MP4,
            video_out,
            physicsClientId=pid
        )

    print("[DEBUG] Beginning replay...")
    cam_dist, cam_yaw, cam_pitch = 0.3, -45, -30

    for i in range(n):
        x,y,z = poses[i]
        r1,r2,r3 = rodr[i]
        inv = 1.0/np.sqrt(1 + r1*r1 + r2*r2 + r3*r3)
        q = [r1*inv, r2*inv, r3*inv, inv]  # quaternion

        dt = (times[i] - times[i-1]) if i>0 else 0.0
        print(f"[{i+1}/{n}] pos=({x:.3f},{y:.3f},{z:.3f})  quat={q}  dt={dt:.4f}s")

        # set pose & orientation
        p.resetBasePositionAndOrientation(
            DRONE_ID,
            [x, y, z],
            q,
            physicsClientId=pid
        )

        # camera follow
        drone_pos, _ = p.getBasePositionAndOrientation(
            DRONE_ID, physicsClientId=pid
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=cam_dist,
            cameraYaw=cam_yaw,
            cameraPitch=cam_pitch,
            cameraTargetPosition=drone_pos,
            physicsClientId=pid
        )

        # render
        env.render()

        # pacing
        if replay_freq:
            time.sleep(1.0/replay_freq)
        else:
            if i+1 < n:
                time.sleep(max(0.0, times[i+1] - times[i]))

    # stop video
    if log_id is not None:
        print("[DEBUG] Stopping video recording")
        p.stopStateLogging(log_id, physicsClientId=pid)

    print("[DEBUG] Replay complete; closing in 1s...")
    time.sleep(1.0)
    env.close()
    print("[DEBUG] Done.")

if __name__=="__main__":
    parser = argparse.ArgumentParser(
        description="Replay a drone log with orientation"
    )
    parser.add_argument('log_file', help="Path to .npy log")
    parser.add_argument('--drone',     default=DroneModel("cf2x"),
                        type=DroneModel, choices=DroneModel)
    parser.add_argument('--physics',   default=Physics("pyb"),
                        type=Physics, choices=Physics)
    parser.add_argument('--nogui',     action='store_true',
                        help="Use DIRECT instead of GUI")
    parser.add_argument('--replay_freq', type=float, default=None,
                        help="Replay frequency in Hz (override log timing)")
    parser.add_argument('--video_out', default=None,
                        help="MP4 output path (optional)")
    args = parser.parse_args()

    replay_log(
        args.log_file,
        drone_model=args.drone,
        physics=args.physics,
        gui=not args.nogui,
        replay_freq=args.replay_freq,
        video_out=args.video_out
    )
