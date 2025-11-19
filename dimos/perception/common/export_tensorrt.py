import argparse
from ultralytics import YOLO, FastSAM
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(description='Export YOLO/FastSAM models to different formats')
    parser.add_argument('--model_path', type=str, required=True, help='Path to the model weights')
    parser.add_argument('--model_type', type=str, choices=['yolo', 'fastsam'], required=True, help='Type of model to export')
    parser.add_argument('--precision', type=str, choices=['fp32', 'fp16', 'int8'], default='fp32', help='Precision for export')
    parser.add_argument('--format', type=str, choices=['onnx', 'engine'], default='onnx', help='Export format')
    return parser.parse_args()

def main():
    args = parse_args()
    half = args.precision == 'fp16'
    int8 = args.precision == 'int8'
    # Load the appropriate model
    if args.model_type == 'yolo':

        model = YOLO(args.model_path)
    else:
        model = FastSAM(args.model_path)

    # Export the model
    model.export(format=args.format, half=half, int8=int8)

if __name__ == '__main__':
    main()