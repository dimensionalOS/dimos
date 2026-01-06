# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

from ultralytics import YOLO, FastSAM  # type: ignore[attr-defined]


def parse_args():  # type: ignore[no-untyped-def]
    parser = argparse.ArgumentParser(description="Export YOLO/FastSAM models to different formats")
    parser.add_argument("--model_path", type=str, required=True, help="Path to the model weights")
    parser.add_argument(
        "--model_type",
        type=str,
        choices=["yolo", "fastsam"],
        required=True,
        help="Type of model to export",
    )
    parser.add_argument(
        "--precision",
        type=str,
        choices=["fp32", "fp16", "int8"],
        default="fp32",
        help="Precision for export",
    )
    parser.add_argument(
        "--format", type=str, choices=["onnx", "engine"], default="onnx", help="Export format"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()  # type: ignore[no-untyped-call]
    half = args.precision == "fp16"
    int8 = args.precision == "int8"
    # Load the appropriate model
    if args.model_type == "yolo":
        model: YOLO | FastSAM = YOLO(args.model_path)
    else:
        model = FastSAM(args.model_path)

    # Export the model
    model.export(format=args.format, half=half, int8=int8)


if __name__ == "__main__":
    main()
