# Export a KernelBench level1 bench to an ExecuTorch .pte (fp32, XNNPACK).
# Uses the SAME sizing knobs as the ModelBlaster flow (BENCH_TARGET_MB default,
# BENCH_MAX_ELEMENTS legacy override) so ET and MB shapes match byte-for-byte.
import argparse, os, sys, torch
sys.path.insert(0, os.environ.get(
    "MB_REPO", "/scratch2/dima/misc_sw/FreshScheduler/zephyr-chipyard-sw"))
from torch.export import export
from executorch.exir import to_edge
from executorch.backends.xnnpack.partition.xnnpack_partitioner import XnnpackPartitioner
from modelblaster.pipeline.extract_graph import _load_kernelbench
ap = argparse.ArgumentParser()
ap.add_argument("--bench", required=True); ap.add_argument("--pte", required=True)
a = ap.parse_args()
_maxel = int(os.environ.get("BENCH_MAX_ELEMENTS", "0"))
_tmb = int(os.environ.get("BENCH_TARGET_MB", "256"))
_tgt = None if _maxel else _tmb * 2**20
m, s, name = _load_kernelbench(a.bench, max_elements=(_maxel or None), target_bytes=_tgt)
args = (s,) if torch.is_tensor(s) else tuple(s)
edge = to_edge(export(m, args)).to_backend(XnnpackPartitioner())
gm = edge.exported_program().graph_module
deleg = sum("call_delegate" in str(n.target) for n in gm.graph.nodes if n.op=="call_function")
aten = sum(("aten" in str(getattr(n.target,'_op',n.target)) and "call_delegate" not in str(n.target)) for n in gm.graph.nodes if n.op=="call_function")
prog = edge.to_executorch()
with open(a.pte, "wb") as f: prog.write_to_file(f)
print(f"wrote {a.pte}  bench={name} delegates={deleg} undelegated_aten={aten} pte_bytes={len(prog.buffer)}")
