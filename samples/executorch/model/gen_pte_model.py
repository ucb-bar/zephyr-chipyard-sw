"""General torchvision -> ExecuTorch .pte exporter (fp32 or int8), for the
flow-comparison harness. Supports the shared models both flows can run.

int8 uses the XNNPACK pt2e quantizer (symmetric, per-channel) — the ExecuTorch
counterpart to modelblaster's int8 PTQ. Cycle comparison is architecture-
determined, so random weights are fine; match the modelblaster config
(mobilenet_v2: width_mult=1.0, 224x224, 1000 classes).
"""
import argparse
import torch
from torch.export import export, ExportedProgram
from executorch.exir import to_edge, to_edge_transform_and_lower, EdgeProgramManager
from executorch.backends.xnnpack.partition.xnnpack_partitioner import XnnpackPartitioner


def build_model(name):
    if name == "lenet":
        class LeNet(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.conv1 = torch.nn.Conv2d(1, 6, 5, 1); self.pool = torch.nn.MaxPool2d(2, 2)
                self.conv2 = torch.nn.Conv2d(6, 16, 5, 1)
                self.fc1 = torch.nn.Linear(16*4*4, 120); self.fc2 = torch.nn.Linear(120, 84); self.fc3 = torch.nn.Linear(84, 10)
            def forward(self, x):
                x = self.pool(torch.relu(self.conv1(x))); x = self.pool(torch.relu(self.conv2(x)))
                x = x.view(x.size(0), -1)
                return self.fc3(torch.relu(self.fc2(torch.relu(self.fc1(x)))))
        return LeNet().eval(), (torch.randn(1, 1, 28, 28),)
    if name == "mobilenet_v2":
        import torchvision.models as tv
        # Match modelblaster mobilenet_v2 at the standard full config.
        m = tv.mobilenet_v2(weights=None, width_mult=1.0, num_classes=1000).eval()
        return m, (torch.randn(1, 3, 224, 224),)
    if name == "dronet":
        # Import the SAME DroNet class modelblaster uses, so both flows run the
        # identical architecture (cycles are architecture-determined). DroNet uses
        # plain ReLU (XNNPACK/pt2e-friendly) — unlike MobileNetV2's ReLU6, which
        # modelblaster's int8 extractor doesn't support.
        import sys
        sys.path.insert(0, "/scratch2/dima/misc_sw/FreshScheduler/zephyr-chipyard-sw")
        from modelblaster.models.dronet import get_model, get_sample_input
        return get_model().eval(), (get_sample_input(),)
    raise SystemExit(f"unknown model {name}")


def export_fp32(model, sample):
    edge = to_edge(export(model, sample)).to_backend(XnnpackPartitioner())
    return edge.to_executorch()


def export_int8(model, sample, calib):
    from executorch.backends.xnnpack.quantizer.xnnpack_quantizer import (
        XNNPACKQuantizer, get_symmetric_quantization_config)
    try:
        from torchao.quantization.pt2e.quantize_pt2e import prepare_pt2e, convert_pt2e
    except ImportError:
        from torch.ao.quantization.quantize_pt2e import prepare_pt2e, convert_pt2e
    try:
        from torch.export import export_for_training as _capture
    except ImportError:
        from torch._export import capture_pre_autograd_graph as _capture
    cap = _capture(model, sample)
    cap = cap.module() if hasattr(cap, "module") else cap
    q = XNNPACKQuantizer().set_global(get_symmetric_quantization_config(is_per_channel=True))
    prep = prepare_pt2e(cap, q)
    for x in calib:
        prep(x)
    quant = convert_pt2e(prep)
    ep = export(quant, sample)
    return to_edge_transform_and_lower(ep, partitioner=[XnnpackPartitioner()]).to_executorch()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--quant", choices=["fp32", "int8"], default="int8")
    ap.add_argument("--pte", required=True)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    torch.manual_seed(args.seed)
    model, sample = build_model(args.model)
    if args.quant == "int8":
        calib = [torch.randn_like(sample[0]) for _ in range(4)]
        prog = export_int8(model, sample, calib)
    else:
        prog = export_fp32(model, sample)
    with open(args.pte, "wb") as f:
        prog.write_to_file(f)
    print(f"wrote {args.pte}  (model={args.model} quant={args.quant} pte_bytes={len(prog.buffer)})")
