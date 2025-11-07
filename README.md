## Overview

A collection of sample workloads and examples to be used with the Zephyr flow for Chipyard.


## Chipyard Installation

TODO

## Standalone Installation

### Main Installation

First, clone this repo:
```
git clone git@github.com:ucb-bar/zephyr-chipyard-sw.git
cd zephyr-chipyard-sw
git submodule update --init
```

Next, install conda and dependencies:
```
source scripts/install_conda.sh
source scripts/install_submodules.sh
```


Install GCC15.1, patched for Zephyr support:
```
bash scripts/install_toolchain.sh
```

Finally, set your environment variables:
```
source scripts/set_envvars.sh
```

To test an example with spike (spike requires a chipyard install):
```
west build -p -b spike_riscv64 samples/hello_world/
spike build/zephyr/zephyr.elf
```


### Executorch Installation
After the main installation, run the following:

```
# Install executorch dependencies
python -m pip install executorch==0.5.0 zstd

# Setup Executorch repos
cd zephyr-chipyard-sw

cd ./third-party/executorch
git checkout zephyr
git submodule sync
git submodule update --init
cd -

cd ./third-party/executorch/backends/xnnpack/third-party/XNNPACK
# git checkout zephyr
git checkout e1515295a8fbd3a90a7264facc3703ae5c4463be # TODO have branch name
cd -

cd ./third-party/executorch/
./install_requirements.sh
cd -
```
./install_requirements.sh --pybind xnnpack # TODO just needs to install python deps, okay if there are CUDA errors

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

