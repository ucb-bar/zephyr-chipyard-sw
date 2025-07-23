## Overview

A collection of sample workloads and examples to be used with the Zephyr flow for Chipyard.


## Chipyard Installation

TODO

## Standalone Installation

### Main Installation
To use this repo, first install the Chipyard Zephyr fork, and then install this samples repo:

```
# Create your zephyr workspace
mkdir zephyr_ws

# Clone Chipyard's Zephyr fork
git clone git@github.com:ucb-bar/zephyr.git
cd zephyr
git checkout dev

# Create a conda environment
conda create -yn zephyr python=3.12
conda activate zephyr

# install the west dependencies
pip3 install west pyelftools

# Initialize west workspace
west init -l .
west config manifest.file west-riscv.yml
west update

# Return to zephyr_ws
cd -

# Install this repository
git clone git@github.com:ucb-bar/zephyr-chipyard-sw.git
cd zephyr-chipyard-sw
git checkout dev
cd -

```

Next, set up your environment variables. The example below is for providing your own cross-compiler:

```
cd zephyr
export ZEPHYR_BASE=$(pwd)
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
# set based on your RISCV toolchain path
export CROSS_COMPILE=/path/to/toolchain/bin/riscv64-unknown-elf-
cd -
```

To test an example with spike:
```
cd zephyr-chipyard-sw
west build -p -b spike_riscv64 samples/hello_world/
spike build/zephyr/zephyr.elf
cd -
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
./install_requirements.sh --pybind xnnpack # TODO just needs to install python deps, okay if there are CUDA errors
cd -
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

