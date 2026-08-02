# Batch-export KernelBench level1 benches to ExecuTorch .pte at a chosen working-
# set size, and compute the host golden checksum from the SAME model instance that
# gets exported (so the baked weights match the .pte). This is the multi-model,
# golden-producing companion to gen_pte_kb.py (which exports a single bench).
#
# Sizing uses the same BENCH_TARGET_MB knob as the ModelBlaster flow: the target
# is total input+output bytes, shrunk from the KernelBench GPU-sized defaults so
# it fits Zephyr's RAM region. 4MB gives non-trivial work while running fast on
# spike; 64MB matches the FireSim sweep; 256MB is the ModelBlaster default.
#
# The golden mirrors the on-target runner EXACTLY: input = all 1.0, output
# checksum = double-precision sum of all output floats (see riscv_executor_runner
# .cpp run_one_pte). Deterministic (fixed seed) so .pte weights + golden repro.
#
# Usage:
#   python gen_pte_kb_sized.py --out-dir DIR \
#       --bench 56_conv_standard_2D__asymmetric_input__asymmetric_kernel [--bench ...]
#   BENCH_TARGET_MB=4 python gen_pte_kb_sized.py --out-dir DIR --bench ...
# Writes DIR/pte/<tag>.pte + DIR/refs.csv (tag,bench,host_checksum,numel,in/out shape,io_mb).
import argparse, os, sys, csv, torch
sys.path.insert(0, os.environ.get(
    "MB_REPO", "/scratch2/dima/misc_sw/FreshScheduler/zephyr-chipyard-sw"))
from torch.export import export
from executorch.exir import to_edge
from executorch.backends.xnnpack.partition.xnnpack_partitioner import XnnpackPartitioner
from modelblaster.pipeline.extract_graph import _load_kernelbench

REPO = os.environ.get("MB_REPO", "/scratch2/dima/misc_sw/FreshScheduler/zephyr-chipyard-sw")
L1 = f"{REPO}/modelblaster/bench/level1"


def bench_path(b):
    """Accept a level1 stem, a filename, or an absolute path."""
    if os.path.isabs(b):
        return b
    if not b.endswith(".py"):
        b += ".py"
    return b if os.path.isfile(b) else f"{L1}/{b}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--bench", action="append", required=True,
                    help="level1 stem / filename / path; repeatable")
    ap.add_argument("--target-mb", type=int,
                    default=int(os.environ.get("BENCH_TARGET_MB", "4")))
    ap.add_argument("--target-gflops", type=float,
                    default=float(os.environ.get("BENCH_TARGET_GFLOPS", "0")),
                    help="if >0, size each bench to this forward-FLOP budget "
                         "(bounds COMPUTE, not just io); target-mb stays an io "
                         "ceiling. Fixes large-K matmuls / 3D convs that have "
                         "tiny io but huge FLOPs.")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    tgt = a.target_mb * 2**20
    tflops = int(a.target_gflops * 1e9) if a.target_gflops > 0 else None
    os.makedirs(f"{a.out_dir}/pte", exist_ok=True)

    rows = []
    for b in a.bench:
        bf = bench_path(b)
        if not os.path.isfile(bf):
            print(f"SKIP {b}: not found ({bf})", file=sys.stderr); continue
        torch.manual_seed(a.seed)
        m, s, name = _load_kernelbench(bf, target_bytes=tgt, target_flops=tflops)
        m = m.eval()
        args = (s,) if torch.is_tensor(s) else tuple(s)
        ones = tuple(torch.ones_like(x) for x in args)
        with torch.no_grad():
            out = m(*ones)
        o0 = (out if isinstance(out, (list, tuple)) else (out,))[0]
        golden, numel = float(o0.double().sum().item()), int(o0.numel())
        in_shape = "x".join(map(str, args[0].shape))
        out_shape = "x".join(map(str, o0.shape))
        edge = to_edge(export(m, args)).to_backend(XnnpackPartitioner())
        gm = edge.exported_program().graph_module
        deleg = sum("call_delegate" in str(n.target)
                    for n in gm.graph.nodes if n.op == "call_function")
        aten = sum(("aten" in str(getattr(n.target, "_op", n.target))
                    and "call_delegate" not in str(n.target))
                   for n in gm.graph.nodes if n.op == "call_function")
        prog = edge.to_executorch()
        pte = f"{a.out_dir}/pte/{name}.pte"
        with open(pte, "wb") as f:
            prog.write_to_file(f)
        io_mb = (args[0].numel() + numel) * 4 / 2**20
        print(f"{name}: in={in_shape} out={out_shape} io={io_mb:.1f}MB "
              f"deleg={deleg} aten={aten} numel={numel} golden={golden:.6f} pte={len(prog.buffer)}B")
        rows.append({"tag": name, "bench": os.path.basename(bf), "host_checksum": f"{golden:.6f}",
                     "numel": numel, "in_shape": in_shape, "out_shape": out_shape,
                     "io_mb": f"{io_mb:.1f}", "delegates": deleg, "undelegated_aten": aten})

    if not rows:
        print("no benches exported", file=sys.stderr); return 1
    with open(f"{a.out_dir}/refs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\nwrote {a.out_dir}/refs.csv ({len(rows)} benches, target {a.target_mb}MB) + {a.out_dir}/pte/*.pte")
    return 0


if __name__ == "__main__":
    sys.exit(main())
