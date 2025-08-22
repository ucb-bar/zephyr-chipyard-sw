import json
import numpy as np

# === CONFIGURATION ===
# force_limit = [4.0, 4.0, -4.0]      # Max force per axis in Newtons
force_limit = [4.0, 4.0, -4.0]      # Max force per axis in Newtons
# torque_limit = [0.01, 0.01, 0.02]  # Max torque per axis in Nm
torque_limit = [0.02, 0.02, 0.04]  # Max torque per axis in Nm
# num_axis_steps = 11                # Number of linearly spaced steps (including 0)
num_axis_steps = 17                # Number of linearly spaced steps (including 0)
num_gust_whack_scales = 11         # Number of scaled variations from 0 to -1.5×

impulse_scale = 10  # Scale for impulse forces

# Templates for gust and whack
gust_template = [
    [0, 1.0,  0, 0, 0, -0.005, 0.75],
    [0, 0,    0, 0, 0, 0,      0.85]
]
whack_template = [
    [-10, 20, -20, 0.0, 0.00, 0.01, 0.75],
    [0,    0,   0, 0,   0,    0,    0.755]
]

dist_dict = {}

def add_entry(name, vec, t, duration, is_torque=False):
    stop_t = t + duration
    if is_torque:
        entry = [
            [0.0, 0.0, 0.0] + list(vec) + [t],
            [0.0]*6 + [stop_t]
        ]
    else:
        entry = [
            list(vec) + [0.0, 0.0, 0.0] + [t],
            [0.0]*6 + [stop_t]
        ]
    dist_dict[name] = entry

# # Axis-aligned sustained forces (100ms)
# for i in [2]:
#     axis = ['x', 'y', 'z'][i]
#     for mag in np.linspace(0, force_limit[i], num_axis_steps):
#         vec = [0.0, 0.0, 0.0]
#         vec[i] = mag
#         name = f"force_{axis}_sust_{mag:+.3f}"
#         add_entry(name, vec, 0.75, 0.1)

# # Axis-aligned impulse forces (5ms)
# for i in [2]:
#     axis = ['x', 'y', 'z'][i]
#     for mag in np.linspace(0, force_limit[i], num_axis_steps):
#         vec = [0.0, 0.0, 0.0]
#         vec[i] = mag * impulse_scale
#         name = f"force_{axis}_impulse_{mag:+.3f}"
#         add_entry(name, vec, 0.75, 0.005)

# Reference
vec = [0.0, 0.0, 0.0]
name = f"ref"
add_entry(name, vec, 0.75, 0.1)

# Axis-aligned sustained forces (100ms)
for i in range(3):
    axis = ['x', 'y', 'z'][i]
    for mag in np.linspace(0, force_limit[i], num_axis_steps):
        if mag == 0.0:
            continue
        vec = [0.0, 0.0, 0.0]
        vec[i] = mag
        name = f"force_{axis}_sust_{mag:+.3f}"
        add_entry(name, vec, 0.75, 0.1)

# Axis-aligned impulse forces (5ms)
for i in range(3):
    axis = ['x', 'y', 'z'][i]
    for mag in np.linspace(0, force_limit[i], num_axis_steps):
        if mag == 0.0:
            continue
        vec = [0.0, 0.0, 0.0]
        mag = mag * impulse_scale
        vec[i] = mag
        name = f"force_{axis}_impulse_{mag:+.3f}"
        add_entry(name, vec, 0.75, 0.005)

# Axis-aligned sustained torques (100ms)
for i in range(3):
    axis = ['x', 'y', 'z'][i]
    for mag in np.linspace(0, torque_limit[i], num_axis_steps):
        if mag == 0.0:
            continue
        vec = [0.0, 0.0, 0.0]
        vec[i] = mag
        name = f"torque_{axis}_sust_{mag:+.5f}"
        add_entry(name, vec, 0.75, 0.1, is_torque=True)

# Axis-aligned impulse torques (5ms)
for i in range(3):
    axis = ['x', 'y', 'z'][i]
    for mag in np.linspace(0, torque_limit[i], num_axis_steps):
        if mag == 0.0:
            continue
        vec = [0.0, 0.0, 0.0]
        mag = mag * impulse_scale
        vec[i] = mag
        name = f"torque_{axis}_impulse_{mag:+.5f}"
        add_entry(name, vec, 0.75, 0.005, is_torque=True)

# Scaled gusts
for s in np.linspace(0, 1.5, num_gust_whack_scales):
    if s == 0.0:
        continue
    scaled = [[s*x for x in row[:6]] + [row[6]] for row in gust_template]
    name = f"gust_scale_{s:+.2f}"
    dist_dict[name] = scaled

# Scaled whacks
for s in np.linspace(0, 1.5, num_gust_whack_scales):
    if s == 0.0:
        continue
    scaled = [[s*x for x in row[:6]] + [row[6]] for row in whack_template]
    name = f"whack_scale_{s:+.2f}"
    dist_dict[name] = scaled

# Save
with open("disturbances/generated_disturbances.json", "w") as f:
    json.dump(dist_dict, f, indent=2)
print("Saved to disturbances/generated_disturbances.json")
