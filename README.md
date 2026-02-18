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


### Executorch/Torch Installation
**Note:** The torch setup uses the `torch-bump-testing` branch of the main repository, which references specific commits for `zephyr_ws/zephyr`, `third-party/executorch`, and `third-party/XNNPACK` submodules.

After the main installation, checkout the torch-bump-testing branch:

```
git fetch origin
git checkout torch-bump-testing
```

Then run the torch installation script:

```
bash scripts/install_torch.sh
```

To test an example using Executorch, inside `zephyr-chipyard-sw`:

```
# Generate executorch C headers
./samples/executorch/generate_pte.sh --model mobilenetv3small

# Build with the RVV XNNPACK Runtime
# Note: Currently requires a patched version of RISCV toolchain
west build -p -b spike_riscv64 ./samples/executorch/executor_runner/ -DXNNPACK_ENABLE_RISCV_VECTOR=ON -DXNNPACK_ENABLE_RISCV_GEMMINI=OFF

# Run using spike
spike -p4 --isa=rv64gcv_zicntr build/zephyr/zephyr.elf
```

## Chipyard Installation

TODO

## Troubleshooting

If you encounter issues during installation or when activating the conda environment, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for detailed troubleshooting information.
