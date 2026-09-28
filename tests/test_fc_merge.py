"""Host checks for the FC merge's command transport and mode conflicts.

Run with ``python3 -m unittest discover -s tests -p test_fc_merge.py -v``.
Compile the actual parser and mode guards from main.cpp with host UART stubs;
these checks require no Zephyr SDK or hardware. They verify command delivery
and configuration isolation, not peripheral behavior or control-loop timing.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "samples/rose_flight_controller/src/main.cpp"


class FlightControllerMerge(unittest.TestCase):
    def test_uart_commands_and_receive_recovery(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("host C++ compiler unavailable")
        source = MAIN.read_text()
        start = source.index("static void uart_cmd_reply(")
        end = source.index("#endif /* ROSE_UART_CMD */", start)
        parser = source[start:end]
        harness = r'''
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#define MIN(a, b) std::min(a, b)
#define printk(...) ((void)0)
static void *esp_uart_dev = nullptr;
static std::deque<unsigned char> rx;
static std::string tx;
static int estop_count, disarm_count, reset_count, profile_count;
static float hover_height = -1;
static int durations[4];
static bool device_is_ready(void *) { return true; }
static int uart_poll_in(void *, unsigned char *ch) {
    if (rx.empty()) { return -1; }
    *ch = rx.front(); rx.pop_front(); return 0;
}
static void esp_uart_write_line(const char *line, size_t size) { tx.append(line, size); }
static void rose_cmd_estop() { estop_count++; }
static void rose_cmd_disarm() { disarm_count++; }
static void rose_cmd_reset() { reset_count++; }
static void rose_cmd_set_hover_z(float height) { hover_height = height; }
static void rose_cmd_set_profile(int a, int b, int c, int d) {
    profile_count++;
    durations[0] = a; durations[1] = b; durations[2] = c; durations[3] = d;
}
'''
        checks = r'''
static void feed(const std::string &text) {
    for (unsigned char ch : text) { rx.push_back(ch); }
    while (!rx.empty()) { uart_cmd_poll(); }
}
int main() {
    feed("ESTOP\r\nDISARM \nRESET\nPING\n");
    assert(estop_count == 1 && disarm_count == 1 && reset_count == 1);
    assert(tx == "FCACK ESTOP\nFCACK DISARM\nFCACK RESET\nFCACK PING\n");
    tx.clear();
    feed("HOVER_Z 250\nPROFILE 100 200 300 400\n");
    assert(hover_height == 0.25f && profile_count == 1);
    assert(durations[0] == 100 && durations[1] == 200 && durations[2] == 300 && durations[3] == 400);
    assert(tx == "FCACK HOVER_Z\nFCACK PROFILE\n");
    tx.clear();
    feed("HOVER_Z\nPROFILE 100 200\nBOGUS\n");
    assert(tx == "FCNAK HOVER_Z\nFCNAK PROFILE\nFCNAK ?\n");
    tx.clear();
    // The shared panel can request SNAP, but this transport cannot capture it.
    // It must explicitly reject it rather than reporting successful execution.
    feed("SNAP\nSNAP 128 126\n");
    assert(tx == "FCNAK SNAP\nFCNAK SNAP\n");
    tx.clear();
    feed("ES"); assert(estop_count == 1);
    feed("TOP\n"); assert(estop_count == 2);
    tx.clear();
    // Never treat the suffix of an overlong line as a command.
    feed(std::string(90, 'X') + "ESTOP\nPING\n");
    assert(estop_count == 2 && tx == "FCACK PING\n");
    tx.clear();
    // A nonprintable reset glitch must not swallow the next stop command.
    feed(std::string("junk") + char(1) + "ESTOP\n");
    assert(estop_count == 3 && tx == "FCACK ESTOP\n");
}
'''
        with tempfile.TemporaryDirectory(prefix="fc-merge-test-") as temp:
            cpp = Path(temp) / "commands.cpp"
            executable = Path(temp) / "commands"
            cpp.write_text(harness + parser + checks)
            result = subprocess.run(
                [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                 str(cpp), "-o", str(executable)], capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_transport_and_camera_mode_isolation(self):
        preprocessor = shutil.which("cpp")
        if preprocessor is None:
            self.skipTest("host C preprocessor unavailable")
        source = MAIN.read_text()
        start = source.index("#ifndef ROSE_UART_CMD")
        uart_guard = source[start:source.index("\n/*", start)]
        start = source.index("#ifndef ROSE_CAMERA\n")
        camera_guard = source[start:source.index("#if ROSE_CAMERA\n", start)]
        names = ("ROSE_UART_TELEM", "ROSE_ESP_MOTORS", "ESP_UART_IS_CONSOLE",
                 "HAVE_ESP_UART", "ROSE_CAMERA", "ROSE_CAMERA_DMA")
        # Expected command-reader default is None for rejected configurations.
        cases = (
            ("driver UART", (0, 0, 0, 1, 0, 0), True, 1),
            ("raw UART", (1, 0, 0, 1, 0, 0), True, 0),
            ("raw UART plus motors", (1, 1, 0, 1, 0, 0), False, None),
            ("raw UART plus console", (1, 0, 1, 1, 0, 0), False, None),
            ("raw UART plus explicit reader", (1, 0, 0, 1, 0, 0), False, None),
            ("DMA capture", (0, 0, 0, 0, 0, 1), True, 0),
            ("still capture", (0, 0, 0, 0, 1, 0), True, 0),
            ("both capture implementations", (0, 0, 0, 0, 1, 1), False, None),
        )
        for name, values, allowed, expected_reader in cases:
            with self.subTest(mode=name):
                definitions = "#define DT_ALIAS(x) x\n#define DT_NODE_HAS_STATUS(x,y) 1\n"
                definitions += "\n".join(
                    f"#define {key} {value}" for key, value in zip(names, values)
                ) + "\n"
                if name == "raw UART plus explicit reader":
                    definitions += "#define ROSE_UART_CMD 1\n"
                text = definitions + uart_guard + camera_guard
                if expected_reader is not None:
                    text += (f"\n#if ROSE_UART_CMD != {expected_reader}\n"
                             "#error Unexpected command-reader default\n#endif\n")
                result = subprocess.run(
                    [preprocessor, "-P", "-"], input=text, capture_output=True, text=True,
                )
                self.assertEqual(result.returncode == 0, allowed, result.stderr)


if __name__ == "__main__":
    unittest.main()
