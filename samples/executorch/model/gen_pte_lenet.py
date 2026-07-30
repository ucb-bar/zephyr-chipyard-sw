"""Standalone LeNet -> ExecuTorch .pte exporter (fp32 or int8).

Torchvision-free variant of gen_pte.py. LeNet is defined inline, matching the
LeNet in gen_pte.py and modelblaster/models/lenet.py (input 1x1x28x28), so the
flow-comparison harness (compare_flows.sh) can put the SAME model through both
flows. int8 uses the XNNPACK pt2e quantizer (symmetric, per-channel) — the
ExecuTorch counterpart to modelblaster's int8 PTQ — for an apples-to-apples
quantized comparison.
"""
import argparse
import torch
from torch.export import export, ExportedProgram
from executorch.exir import to_edge, to_edge_transform_and_lower, EdgeProgramManager
from executorch.backends.xnnpack.partition.xnnpack_partitioner import XnnpackPartitioner


class LeNet(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = torch.nn.Conv2d(1, 6, kernel_size=5, stride=1)
        self.pool = torch.nn.MaxPool2d(2, 2)
        self.conv2 = torch.nn.Conv2d(6, 16, kernel_size=5, stride=1)
        self.fc1 = torch.nn.Linear(16 * 4 * 4, 120)
        self.fc2 = torch.nn.Linear(120, 84)
        self.fc3 = torch.nn.Linear(84, 10)

    def forward(self, x):
        x = torch.nn.functional.relu(self.conv1(x))
        x = self.pool(x)
        x = torch.nn.functional.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(x.size(0), -1)
        x = torch.nn.functional.relu(self.fc1(x))
        x = torch.nn.functional.relu(self.fc2(x))
        x = self.fc3(x)
        return x


def export_fp32(model, sample):
    ep: ExportedProgram = export(model, sample)
    edge: EdgeProgramManager = to_edge(ep)
    edge = edge.to_backend(XnnpackPartitioner())
    return edge.to_executorch()


def export_int8(model, sample, calib):
    # pt2e static PTQ with the XNNPACK quantizer (symmetric, per-channel weights).
    from executorch.backends.xnnpack.quantizer.xnnpack_quantizer import (
        XNNPACKQuantizer, get_symmetric_quantization_config)
    # pt2e moved from torch.ao to torchao in the torch 2.9 line; the executorch
    # XNNPACKQuantizer emits torchao QuantizationSpecs, so use the torchao pt2e.
    try:
        from torchao.quantization.pt2e.quantize_pt2e import prepare_pt2e, convert_pt2e
    except ImportError:
        from torch.ao.quantization.quantize_pt2e import prepare_pt2e, convert_pt2e
    try:
        from torch.export import export_for_training as _capture
    except ImportError:
        from torch._export import capture_pre_autograd_graph as _capture  # older torch

    captured = _capture(model, sample)
    captured = captured.module() if hasattr(captured, "module") else captured
    quantizer = XNNPACKQuantizer().set_global(
        get_symmetric_quantization_config(is_per_channel=True))
    prepared = prepare_pt2e(captured, quantizer)
    for x in calib:                     # calibration
        prepared(x)
    quantized = convert_pt2e(prepared)
    ep: ExportedProgram = export(quantized, sample)
    # to_edge_transform_and_lower handles the quantized-op → XNNPACK lowering.
    edge = to_edge_transform_and_lower(ep, partitioner=[XnnpackPartitioner()])
    return edge.to_executorch()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pte", default="model.pte")
    ap.add_argument("--quant", choices=["fp32", "int8"], default="fp32")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save-io", default=None,
                    help="optional .npz to save {input, output} PyTorch reference")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    model = LeNet().eval()
    sample = (torch.randn(1, 1, 28, 28),)
    with torch.no_grad():
        ref_out = model(*sample)

    if args.quant == "int8":
        calib = [torch.randn(1, 1, 28, 28) for _ in range(8)]
        exec_prog = export_int8(model, sample, calib)
    else:
        exec_prog = export_fp32(model, sample)

    with open(args.pte, "wb") as f:
        exec_prog.write_to_file(f)
    print(f"wrote {args.pte}  (quant={args.quant})")

    if args.save_io:
        import numpy as np
        np.savez(args.save_io,
                 input=sample[0].numpy().astype("float32"),
                 output=ref_out.numpy().astype("float32"))
        print(f"wrote {args.save_io}")
