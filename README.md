## Overview

A collection of sample workloads and examples to be used with the Zephyr flow for Chipyard.

## Standalone Installation

### Zephyr SDK Installation

The Zephyr SDK provides a pre-built toolchain with GNU and LLVM compilers, host tools, and CMake integration. This is an alternative to building the toolchain from source.

First, clone this repo:
```
git clone git@github.com:ucb-bar/zephyr-chipyard-sw.git
cd zephyr-chipyard-sw
git submodule update --init
```

Next, install conda and dependencies:
```
source scripts/install_conda.sh
bash scripts/install_submodules.sh
```

Install the Zephyr SDK (minimal SDK with RISC-V 64-bit support):
```
bash scripts/install_toolchain_sdk.sh
```

Finally, set your environment variables:
```
source scripts/set_envvars_sdk.sh
```

After installation, activate the conda environment:
```
source tools/miniforge3/etc/profile.d/conda.sh
conda activate zephyr
```

To test an example with spike (spike requires a chipyard install):
```
west build -p -b spike_riscv64 samples/hello_world/
spike build/zephyr/zephyr.elf
```

### Zephyr fork branch

`zephyr_ws/zephyr` tracks the **`rose-2-dev`** branch of `ucb-bar/zephyr` (see `.gitmodules`).
It is the reconciled RoSE line: the Saturn/RISC-V fork (V/F save-restore decouple, `uart_htif`
buffered-output flush, POSIX `pthread_attr_*affinity`) **plus** the ST ToF drivers
(`vl53l5cx` multizone, `vl53l1`) used by the riskybird bringup. `git submodule update --init`
checks it out at the recorded commit; the setup above builds against this in-tree kernel (no
external Zephyr checkout needed).

### RoSE bridge / co-sim samples

The `rose_flight_controller` and `rose_nav_controller` samples (IMU/flow/ToF + estimator +
TinyMPC, plus the DroNet vision-nav and timer-driven builds) use the RoSE co-sim bridge module
`zephyr-rose`, which lives in the RoSE super-repo. From a RoSE checkout the one-shot builder is
`soc/sim/build_zephyr_rose.sh` (it activates this in-tree conda + SDK and passes
`-DZEPHYR_EXTRA_MODULES=<...>/zephyr-rose`). To build one directly here:
```
source scripts/activate_conda.sh && source scripts/set_envvars_sdk.sh
west build -p always -b spike_riscv64 samples/rose_nav_controller \
  -- -DZEPHYR_EXTRA_MODULES=<path-to>/soc/sw/zephyr-rose
```
Run the resulting ELF under the RoSE lockstep (`soc/sim/run_spike_rose_lockstep.sh`) with a
synchronizer; see `docs/ROSE_DRONET_INTEGRATION_PLAN.md` in the super-repo.

### Cross Compiler Installation

First, clone this repo:
```
git clone git@github.com:ucb-bar/zephyr-chipyard-sw.git
cd zephyr-chipyard-sw
git submodule update --init
```

Next, install conda and dependencies:
```
source scripts/install_conda.sh
bash scripts/install_submodules.sh
```

Install GCC15.1, patched for Zephyr support:
```
bash scripts/install_toolchain.sh
```

Finally, set your environment variables:
```
source scripts/set_envvars.sh
```

After installation, activate the conda environment:
```
source tools/miniforge3/etc/profile.d/conda.sh
conda activate zephyr
```

To test an example with spike (spike requires a chipyard install):
```
west build -p -b spike_riscv64 samples/hello_world/
spike build/zephyr/zephyr.elf
```


### Executorch Installation

`third-party/executorch` is pinned to the `zephyr-1.0.1` branch of `ucb-bar/executorch`
(ExecuTorch 1.0.1 + the RVV/Zephyr XNNPACK fork). Keep the recorded commit: do **not**
`git checkout zephyr`, which is the old 0.5 line and no longer builds with the samples.
ExecuTorch 1.0.1 also removed `install_requirements.sh --pybind`; the Python side is
installed from the released wheels instead.

After the main installation, with the `zephyr` env active:

```
# Initialize ExecuTorch's nested submodules (XNNPACK fork, flatbuffers, json, ...)
cd ./third-party/executorch
git submodule sync
git submodule update --init --recursive
cd -

# Python deps, matched to the 1.0.1 runtime (torch 2.9 line, CPU wheels)
pip install --index-url https://download.pytorch.org/whl/cpu torch==2.9.0 torchvision==0.24.0
pip install executorch==1.0.1 torchao==0.14.0 zstd
```

`install_submodules.sh` already installs a `cmake>=3.29,<4` into the env; ExecuTorch 1.0.1
needs >= 3.29. Make sure the env's `bin` comes first on `PATH` (`which cmake`), since some
machines have an older system or Vitis cmake ahead of it. The torchao warning
"Skipping import of cpp extensions" is harmless for export.
See `modelblaster/notes/flow_comparison.md` for more detail on the 1.0.1 port.

To test an example using Executorch, inside `zephyr-chipyard-sw`:

```
# Generate executorch C headers
./samples/executorch/generate_pte.sh --model mobilenetv3small

# Build with the RVV XNNPACK Runtime
# (rv64 ABI flags for the ExecuTorch/XNNPACK subbuild are set in the sample's CMakeLists.txt)
west build -p -b spike_riscv64 ./samples/executorch/executor_runner/ -DXNNPACK_ENABLE_RISCV_VECTOR=ON -DXNNPACK_ENABLE_RISCV_GEMMINI=OFF

# Run using spike
spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf
```

## Chipyard Installation

TODO

## Troubleshooting

If you encounter issues during installation or when activating the conda environment, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for detailed troubleshooting information.
