# Troubleshooting

## Installation Issues

### Script crashes or terminal exits

If `install_submodules.sh` crashes your terminal:

1. **Check the log file:**
   ```bash
   cat .install_submodules.log
   ```
   This contains detailed error messages and diagnostics.

2. **Common issues:**
   - Pybullet build failures are handled gracefully - the script will continue with a warning
   - Check that conda is properly installed and in PATH

### Conda activation crashes terminal

If `conda activate zephyr` crashes your terminal:

1. **Debug the issue:**
   ```bash
   bash scripts/debug_conda.sh
   ```
   This will test conda activation safely and save diagnostics to `.debug_conda.log`.

2. **Use the safer activation script:**
   ```bash
   source scripts/activate_conda.sh
   ```

3. **Alternative: Use `conda run` instead of activate:**
   Instead of activating, you can run commands directly:
   ```bash
   conda run -n zephyr <your-command>
   ```

### West command not found

If you get "west: command not found" errors:

- The `install_submodules.sh` script uses `conda run -n zephyr` to run west commands, so they should work during installation
- After installation, make sure you've activated the zephyr environment:
  ```bash
  source tools/miniforge3/etc/profile.d/conda.sh
  conda activate zephyr
  ```
- Or use `conda run -n zephyr west <command>` instead

### Pybullet installation fails

The gym-pybullet-drones installation may fail due to pybullet build errors. This is handled gracefully:

- The installation will continue with a warning
- You can try installing it later manually:
  ```bash
  conda run -n zephyr pip install -e ./tools/gym-pybullet-drones
  ```
- Or activate the environment and install:
  ```bash
  conda activate zephyr
  pip install -e ./tools/gym-pybullet-drones
  ```

## Environment Setup

### Activating the conda environment

After installation, you need to activate the zephyr environment:

```bash
# First, source conda
source tools/miniforge3/etc/profile.d/conda.sh

# Then activate zephyr
conda activate zephyr
```

**If activation crashes your terminal**, see the "Conda activation crashes terminal" section above.

### Using conda run instead of activation

If activation causes issues, you can use `conda run` to execute commands without activating:

```bash
conda run -n zephyr west build -p -b spike_riscv64 samples/hello_world/
conda run -n zephyr python your_script.py
```

## Getting Help

- Check log files:
  - `.install_submodules.log` - Installation logs
  - `.debug_conda.log` - Conda debugging logs (if you ran debug_conda.sh)

- Common issues are documented above. If you encounter other problems, please check:
  1. All prerequisites are installed
  2. Conda is properly initialized
  3. The zephyr environment exists and is accessible

