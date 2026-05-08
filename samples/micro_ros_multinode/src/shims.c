/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-libc shims for the libmicroros colcon build.
 *
 * The riscv64-zephyr-elf SDK 1.0.0-beta1 + picolibc combo doesn't expose
 * isatty() the way rcutils/logging.c expects. Since we don't have a real TTY
 * on spike/FireSim anyway, a permanent "not a tty" stub is correct semantics
 * — color escape codes would just clutter HTIF output.
 */

int isatty(int fd)
{
	(void)fd;
	return 0;
}
