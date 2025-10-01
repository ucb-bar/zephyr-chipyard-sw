import os
import json
import numpy as np
import argparse

def make_random_maneuver(rng,
                         name,
                         n_wp,
                         spatial_scale,
                         time_between,
                         vertical_scale=None,
                         z_min=0.0,
                         z_max=None):
    """
    Generates a single‐drone maneuver:
      - name:            string
      - n_wp:            number of waypoints
      - spatial_scale:   avg horizontal distance between waypoints (m)
      - time_between:    avg time between waypoints (s)
      - vertical_scale:  avg vertical distance per step (m); if None == spatial_scale
      - z_min:           minimum altitude (m)
      - z_max:           maximum altitude (m); if None, unbounded above
    """
    if vertical_scale is None:
        vertical_scale = spatial_scale

    traj = []
    # start at origin at a random allowed altitude
    if z_max is not None:
        z0 = rng.uniform(z_min, z_max)
    else:
        z0 = z_min
    pos = np.array([0.0, 0.0, z0], dtype=float)
    t = 0.0
    traj.append([pos[0], pos[1], pos[2], t])

    for _ in range(1, n_wp):
        # random 3D direction
        d = rng.standard_normal(3)
        d /= np.linalg.norm(d)

        # horizontal & vertical steps
        h_step = spatial_scale * (0.8 + 0.4*rng.random())
        v_step = vertical_scale  * (0.8 + 0.4*rng.random())

        # move
        pos[:2] += d[:2] * h_step
        pos[2]  += np.sign(d[2]) * v_step

        # clamp altitude
        pos[2] = max(pos[2], z_min)
        if z_max is not None:
            pos[2] = min(pos[2], z_max)

        # advance time
        t += time_between * (0.8 + 0.4*rng.random())
        traj.append([pos[0], pos[1], pos[2], t])

    return {"name": name, "traj": traj}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed',    type=int, default=1234,
                        help="Seed for RNG")
    parser.add_argument('--count',   type=int, default=3,
                        help="Number of trajectories per difficulty")
    parser.add_argument('--output',  type=str, default="traj/auto_maneuvers_3d.json",
                        help="Path to write JSON maneuvers")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    # Define your difficulty tiers in one place:
    difficulties = {
        "hard": {
            "n_wp":           10,
            "spatial_scale":  1.1,
            "time_between":   0.3,
            "vertical_scale": 0.5,
            "z_min":          0.5,
            "z_max":          2.0,
        },
        "medium": {
            "n_wp":           7,
            "spatial_scale":  0.7,
            "time_between":   0.4,
            "vertical_scale": 0.5,
            "z_min":          0.5,
            "z_max":          1.8,
        },
        "easy": {
            "n_wp":           5,
            "spatial_scale":  0.3,
            "time_between":   0.5,
            "vertical_scale": 0.5,
            "z_min":          0.5,
            "z_max":          1.5,
        },
    }

    maneuvers = []
    for name, params in difficulties.items():
        for k in range(1, args.count + 1):
            traj_name = f"{name}_{k}"
            man = make_random_maneuver(rng, traj_name, **params)
            # append stabilization waypoint 2s after last
            last = man["traj"][-1]
            x, y, z, t_last = last
            man["traj"].append([x, y, z, t_last + 2.0])
            maneuvers.append(man)

    # Write out
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as fp:
        json.dump(maneuvers, fp, indent=2)

    print(f"Wrote {len(maneuvers)} maneuvers to {args.output} (seed={args.seed})")
