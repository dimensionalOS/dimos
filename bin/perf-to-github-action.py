#!/usr/bin/env python3
import json
import sys
import argparse

def convert(input_file):
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    # github-action-benchmark expects a list of objects:
    # { "name": "...", "unit": "...", "value": ... }
    
    # We can report multiple metrics
    metrics = [
        {
            "name": "Total CPU Time",
            "unit": "s",
            "value": data["total_cpu_s"]
        },
        {
            "name": "User CPU Time",
            "unit": "s",
            "value": data["user_cpu_s"]
        },
        {
            "name": "System CPU Time",
            "unit": "s",
            "value": data["system_cpu_s"]
        },
        {
            "name": "CPU Usage Percentage",
            "unit": "%",
            "value": data["cpu_percentage"]
        }
    ]
    
    print(json.dumps(metrics, indent=2))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Input JSON from measure-perf")
    args = parser.parse_args()
    convert(args.input)
