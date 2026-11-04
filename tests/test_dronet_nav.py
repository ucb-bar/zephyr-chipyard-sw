"""Host checks for DroNet navigation (ROSE_DRONET_NAV) and the yaw-frame controller input.

Run with ``python3 -m unittest discover -s tests -p test_dronet_nav.py -v``.
Compiles the firmware's own fc_dronet_nav.h / fc_shared_math.h and controller_pid.cpp on the host,
and preprocesses main.cpp's mode guards. No Zephyr SDK or hardware; this checks the setpoint math,
the frame transform and the build-mode contract, not inference, camera capture or flight.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "samples/rose_flight_controller/src"
MAIN = SRC / "main.cpp"

# Quaternion helpers shared by the C++ checks: q = qz(yaw) * qy(pitch) * qx(roll) as Gibbs r = q_xyz/q_w.
QUAT = r'''
#include <cmath>
struct Q { float w, x, y, z; };
static Q mul(Q a, Q b) {
    return {a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z, a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x, a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w};
}
static void gibbs(float roll, float pitch, float yaw, float r[3]) {
    Q qx{std::cos(roll/2), std::sin(roll/2), 0, 0}, qy{std::cos(pitch/2), 0, std::sin(pitch/2), 0};
    Q qz{std::cos(yaw/2), 0, 0, std::sin(yaw/2)};
    Q q = mul(qz, mul(qy, qx));
    r[0] = q.x / q.w; r[1] = q.y / q.w; r[2] = q.z / q.w;
}
'''


def compile_and_run(test, source, extra_sources=()):
    compiler = shutil.which("c++")
    if compiler is None:
        test.skipTest("host C++ compiler unavailable")
    with tempfile.TemporaryDirectory(prefix="dronet-nav-test-") as temp:
        cpp = Path(temp) / "check.cpp"
        exe = Path(temp) / "check"
        cpp.write_text(source)
        result = subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{SRC}",
             str(cpp), *map(str, extra_sources), "-o", str(exe), "-lm"],
            capture_output=True, text=True,
        )
        test.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        test.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class DronetNavMath(unittest.TestCase):
    def test_yaw_and_frame_transform(self):
        compile_and_run(self, r'''
#include <cassert>
#include <initializer_list>
#include "fc_shared_math.h"
''' + QUAT + r'''
static bool near(float a, float b, float tol = 1e-5f) { return std::fabs(a - b) < tol; }
int main() {
    // Pure yaw: Gibbs r3 = tan(psi/2).
    for (float psi : {-2.5f, -1.0f, 0.0f, 0.3f, 1.5f, 3.0f}) {
        float r[3] = {0.0f, 0.0f, std::tan(0.5f * psi)};
        assert(near(fc_yaw_from_gibbs(r), psi));
    }
    // Tilted: fc_yaw_from_gibbs is the ZYX yaw.
    float r[3];
    gibbs(0.1f, -0.25f, 0.7f, r);
    assert(near(fc_yaw_from_gibbs(r), 0.7f));
    // At yaw 90 deg the nose points along world +y: world +y velocity is forward, world +x is right.
    float h[2];
    fc_world_to_heading(0.0f, 0.3f, 1.5707963f, h);
    assert(near(h[0], 0.3f) && near(h[1], 0.0f));
    fc_world_to_heading(0.3f, 0.0f, 1.5707963f, h);
    assert(near(h[0], 0.0f) && near(h[1], -0.3f));

    // The yaw-frame transform turns (tilt, yaw psi) into exactly (tilt, yaw 0) -- the review's case:
    // pitch 0.2 rad at yaw 1 rad reads r1 = -0.055 in the raw state, a phantom roll.
    for (float psi : {-1.0f, 0.0f, 1.0f, 2.0f}) {
        float s[12] = {0}, out[12], ref[12] = {0};
        gibbs(0.05f, 0.2f, psi, &s[3]);
        s[6] = std::cos(psi) * 0.3f; s[7] = std::sin(psi) * 0.3f;   // 0.3 m/s along the nose
        s[9] = 0.1f; s[10] = -0.2f; s[11] = 0.3f;
        fc_state_to_yaw_frame(s, psi, out);
        gibbs(0.05f, 0.2f, 0.0f, &ref[3]);
        for (int i = 3; i < 6; i++) assert(near(out[i], ref[i], 1e-5f));
        assert(near(out[6], 0.3f) && near(out[7], 0.0f));
        assert(out[9] == s[9] && out[10] == s[10] && out[11] == s[11]);   // body rates untouched
        if (psi == 1.0f) assert(std::fabs(s[3] - ref[3]) > 0.04f);       // the raw state did mix
    }
    // A yaw error relative to the frame shows up as a small Gibbs yaw, not a large absolute one.
    float s[12] = {0}, out[12];
    gibbs(0.0f, 0.0f, 1.1f, &s[3]);
    fc_state_to_yaw_frame(s, 1.0f, out);
    assert(near(out[5], std::tan(0.05f), 1e-5f));
    // Identity at yaw 0.
    gibbs(0.1f, -0.2f, 0.3f, &s[3]); s[6] = 0.2f; s[7] = -0.1f;
    fc_state_to_yaw_frame(s, 0.0f, out);
    for (int i = 0; i < 12; i++) assert(near(out[i], s[i], 1e-6f));
}
''')

    def test_setpoint_mapping(self):
        compile_and_run(self, r'''
#include <cassert>
#include <cmath>
#include "fc_dronet_nav.h"
#include "fc_shared_math.h"
static bool near(float a, float b, float tol = 1e-4f) { return std::fabs(a - b) < tol; }
static fc_dronet_nav_params params(float coll_scale) {
    fc_dronet_nav_params p;
    p.steer_scale = 1.0f / 127.0f; p.coll_scale = coll_scale;
    p.v_max = 0.3f; p.yaw_rate_max = 0.5f; p.yaw_sign = 1.0f;
    p.tau_v = 0.5f; p.tau_w = 0.3f; p.yaw_max = 1.0f; p.timeout_ms = 600;
    return p;
}
static fc_dronet_nav_out run(fc_dronet_nav *n, const fc_dronet_nav_params &p, bool engaged,
                             bool have, int64_t age, int8_t s, int8_t c, float seconds) {
    fc_dronet_nav_out o{};
    for (int i = 0; i < (int)(seconds / 0.005f); i++) o = fc_dronet_nav_step(n, &p, engaged, have, age, s, c, 0.005f);
    return o;
}
int main() {
    const fc_dronet_nav_params p = params(1.0f / 128.0f);
    fc_dronet_nav n;
    fc_dronet_nav_reset(&n);

    // No command yet: hover, heading held.
    fc_dronet_nav_out o = run(&n, p, true, false, 0, 0, 0, 2.0f);
    assert(!o.fresh && o.vx == 0.0f && o.yaw_gibbs == 0.0f);

    // Clear path, straight ahead: forward speed settles at v_max, no yaw.
    o = run(&n, p, true, true, 50, 0, 0, 5.0f);
    assert(o.fresh && near(o.vx, 0.3f, 1e-3f) && o.yaw_gibbs == 0.0f);
    // Collision probability 0.5 halves it.
    o = run(&n, p, true, true, 50, 0, 64, 5.0f);
    assert(near(o.vx, 0.15f, 1e-3f));
    // A saturated (127) or impossible negative collision output reads as a collision: stop.
    o = run(&n, p, true, true, 50, 0, 127, 5.0f);
    assert(o.collision == 1.0f && near(o.vx, 0.0f, 1e-3f));
    o = run(&n, p, true, true, 50, 0, 0, 5.0f);
    o = run(&n, p, true, true, 50, 0, -100, 5.0f);
    assert(o.collision == 1.0f && near(o.vx, 0.0f, 1e-3f));
    // With the committed gray graph's scale the largest unsaturated output is 0.400: saturation is
    // the only way that model can command a stop.
    const fc_dronet_nav_params g = params(0.0031727139f);
    fc_dronet_nav_reset(&n);
    o = run(&n, g, true, true, 50, 0, 126, 5.0f);
    assert(near(o.collision, 0.39976f, 1e-4f) && near(o.vx, 0.3f * (1.0f - 0.39976f), 1e-3f));
    o = run(&n, g, true, true, 50, 0, 127, 5.0f);
    assert(near(o.vx, 0.0f, 1e-3f));

    // A stale command inside the engaged state decays smoothly and holds the heading reached.
    fc_dronet_nav_reset(&n);
    run(&n, p, true, true, 50, 64, 0, 1.0f);
    const float psi_before = n.psi_ref;
    assert(psi_before > 0.0f);
    o = fc_dronet_nav_step(&n, &p, true, true, 601, 64, 0, 0.005f);
    assert(!o.fresh && o.vx > 0.2f);                                   // not a snap
    o = run(&n, p, true, true, 601, 64, 0, 5.0f);
    assert(near(o.vx, 0.0f, 1e-3f) && std::fabs(n.w) < 1e-3f);
    assert(n.psi_ref >= psi_before && n.psi_ref - psi_before < 0.2f);   // only the decaying tail
    // A negative age (clock skew) is not fresh either.
    o = fc_dronet_nav_step(&n, &p, true, true, -1, 0, 0, 0.005f);
    assert(!o.fresh);

    // Positive steer yaws nose-left (+z), saturates at |steer| = 1 and at yaw_max.
    fc_dronet_nav_reset(&n);
    run(&n, p, true, true, 50, 127, 0, 1.0f);
    assert(n.w > 0.0f && n.w <= 0.5f + 1e-6f);
    o = run(&n, p, true, true, 50, 127, 0, 10.0f);
    assert(near(n.psi_ref, 1.0f) && near(o.yaw_gibbs, std::tan(0.5f)));
    fc_dronet_nav_params flipped = p;
    flipped.yaw_sign = -1.0f;
    fc_dronet_nav_reset(&n);
    run(&n, flipped, true, true, 50, 127, 0, 1.0f);
    assert(n.psi_ref < 0.0f);

    // Leaving the engaged state zeroes the request on the SAME tick and holds the heading.
    fc_dronet_nav_reset(&n);
    run(&n, p, true, true, 50, 64, 0, 2.0f);
    const float held = n.psi_ref;
    assert(n.v > 0.1f);
    o = fc_dronet_nav_step(&n, &p, false, true, 50, 64, 0, 0.005f);
    assert(o.vx == 0.0f && n.w == 0.0f && n.psi_ref == held);
    o = run(&n, p, false, true, 50, 127, 0, 5.0f);
    assert(o.vx == 0.0f && n.psi_ref == held);

    // dt <= 0 changes nothing while engaged.
    fc_dronet_nav before = n;
    fc_dronet_nav_step(&n, &p, true, true, 50, 127, 0, 0.0f);
    assert(n.v == before.v && n.w == before.w && n.psi_ref == before.psi_ref);

    // The hover window is exactly where the profile holds the hover height.
    const int tc = 1500, th = 2500, td = 1500;
    for (int64_t t = 0; t < tc + th + td; t += 5) {
        const bool hover = fc_profile_in_hover(t, tc, th);
        const bool flat = fc_profile_z(t, tc, th, td, 0.8f, -0.2f) == 0.8f;
        // t = tc + th is the descent's first sample, which still reads the hover height.
        assert(hover == (flat && t != tc + th));
    }
    assert(!fc_profile_in_hover(0, 0, 100) && fc_profile_in_hover(1, 0, 100));
    assert(!fc_profile_in_hover(10, 0, 0));
}
''')


class YawFrameController(unittest.TestCase):
    def test_pid_command_is_yaw_invariant_in_its_yaw_frame(self):
        """Same tilt, rates and nose-relative velocity at any heading must give the same command."""
        compile_and_run(self, r'''
#include <cassert>
#include <cstdio>
#include "controller_pid.hpp"
#include "fc_shared_math.h"
''' + QUAT + r'''
// What solve_control() does under ROSE_VEL_HEADING_FRAME, on a controller carried across ticks.
static void run(float psi, float psi_ref, bool transform, float u[4]) {
    HierarchicalPidController c;
    c.init();
    float s[12] = {0}, sp[12] = {0};
    s[2] = 0.5f;
    for (int i = 0; i < 80; i++) {
        const float roll = 0.03f * std::sin(0.1f * i), pitch = 0.15f + 0.02f * std::cos(0.13f * i);
        gibbs(roll, pitch, psi, &s[3]);
        const float vf = 0.1f + 0.002f * i, vl = -0.02f;   // nose-relative velocity
        s[6] = std::cos(psi) * vf - std::sin(psi) * vl;
        s[7] = std::sin(psi) * vf + std::cos(psi) * vl;
        s[9] = 0.05f; s[10] = -0.1f; s[11] = 0.02f;
        sp[2] = 0.5f; sp[5] = std::tan(0.5f * psi_ref); sp[6] = 0.3f;
        if (transform) {
            float st[12], spp[12];
            const float yaw = fc_yaw_from_gibbs(&s[3]);
            float dyaw = 2.0f * std::atan(sp[5]) - yaw;
            dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
            fc_state_to_yaw_frame(s, yaw, st);
            for (int k = 0; k < 12; k++) spp[k] = sp[k];
            spp[5] = std::tan(0.5f * dyaw);
            c.compute(st, spp, u, 0.005f);
        } else {
            c.compute(s, sp, u, 0.005f);
        }
    }
}
static float diff(const float a[4], const float b[4]) {
    float d = 0; for (int i = 0; i < 4; i++) d += std::fabs(a[i] - b[i]); return d;
}
int main() {
    float a[4], b[4], c[4], d[4], e[4];
    run(0.0f, 0.0f, true, a);          // heading 0: the case the PID was tuned on
    run(1.0f, 1.0f, true, b);          // yawed 57 deg, pitched, integrators and slew evolving
    run(-0.8f, -0.8f, true, c);
    assert(diff(a, b) < 1e-4f && diff(a, c) < 1e-4f);
    // Without the transform the yawed case differs: velocity and tilt are read in the wrong frame.
    run(1.0f, 1.0f, false, d);
    assert(diff(a, d) > 1e-3f);
    // At heading 0 the transform is the identity: same as the untransformed controller.
    run(0.0f, 0.0f, false, e);
    assert(diff(a, e) < 1e-6f);
    // A yaw error against the reference is still corrected (the yaw loop sees it).
    float f[4];
    run(1.1f, 1.0f, true, f);
    assert(diff(a, f) > 1e-4f);
    // Yaw lagging its target by the same amount gives the same command at any heading: the frame
    // follows the body, and only the yaw loop sees the error.
    float g0[4], g1[4];
    run(0.0f, 1.0f, true, g0);
    run(-1.0f, 0.0f, true, g1);
    assert(diff(g0, g1) < 1e-4f);
}
''', extra_sources=(SRC / "controller_pid.cpp",))


class DronetNavModeGuards(unittest.TestCase):
    def test_mode_guards(self):
        preprocessor = shutil.which("cpp")
        if preprocessor is None:
            self.skipTest("host C preprocessor unavailable")
        source = MAIN.read_text()
        start = source.index("#ifndef ROSE_DRONET_NAV\n")
        guards = source[start:source.index("#if ROSE_DRONET_NAV\n#include", start)]
        names = ("ROSE_HAVE_DRONET", "ROSE_FLOW", "ROSE_CAMERA_DMA", "ROSE_AUTOFLIGHT",
                 "ROSE_MOTORS_INHIBIT", "DRONET_MODEL_VALIDATED", "ROSE_USE_ROCC",
                 "ROSE_POS_LOOP", "ROSE_VEL_LOOP")
        flight = (1, 1, 1, 1, 0, 1, 0, 0, 1)
        cases = (
            ("nav flight build, validated model", 1, None, flight, True, 1),
            ("nav flight build, benchmark model", 1, None, (1, 1, 1, 1, 0, 0, 0, 0, 1), False, None),
            ("nav bench build, motors inhibited", 1, None, (1, 1, 1, 0, 1, 0, 0, 0, 1), True, 1),
            ("nav autoflight, motors inhibited", 1, None, (1, 1, 1, 1, 1, 0, 0, 0, 1), True, 1),
            ("nav outside rose_fc_dronet", 1, None, (0, 1, 1, 1, 0, 1, 0, 0, 1), False, None),
            ("nav without flow", 1, None, (1, 0, 1, 1, 0, 1, 0, 0, 1), False, None),
            ("nav without camera DMA", 1, None, (1, 1, 0, 1, 0, 1, 0, 0, 1), False, None),
            ("nav, no autoflight, motors live", 1, None, (1, 1, 1, 0, 0, 1, 0, 0, 1), False, None),
            ("nav on the RoCC controller", 1, None, (1, 1, 1, 1, 0, 1, 1, 0, 1), False, None),
            ("nav with the position loop", 1, None, (1, 1, 1, 1, 0, 1, 0, 1, 1), False, None),
            ("nav with the velocity loop off", 1, None, (1, 1, 1, 1, 0, 1, 0, 0, 0), False, None),
            ("nav, heading frame explicitly off", 1, 0, flight, False, None),
            ("existing builds unchanged", 0, None, (0, 0, 0, 0, 0, 0, 1, 1, 0), True, 0),
            ("heading frame alone", 0, 1, (0, 0, 0, 0, 0, 0, 0, 0, 1), True, 1),
            ("heading frame alone on RoCC", 0, 1, (0, 0, 0, 0, 0, 0, 1, 0, 1), False, None),
        )
        for name, nav, heading, values, allowed, expected_heading in cases:
            with self.subTest(mode=name):
                text = f"#define ROSE_DRONET_NAV {nav}\n"
                if heading is not None:
                    text += f"#define ROSE_VEL_HEADING_FRAME {heading}\n"
                text += "\n".join(f"#define {k} {v}" for k, v in zip(names, values)) + "\n"
                text += guards
                if expected_heading is not None:
                    text += (f"\n#if ROSE_VEL_HEADING_FRAME != {expected_heading}\n"
                             "#error unexpected heading-frame default\n#endif\n")
                result = subprocess.run([preprocessor, "-P", "-"], input=text,
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode == 0, allowed, result.stderr)


if __name__ == "__main__":
    unittest.main()
