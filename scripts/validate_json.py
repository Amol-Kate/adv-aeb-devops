# validate_json.py

# Purpose:
# Validates JSON files used for scenarios, KPIs, or configuration.

import json
import sys
from pathlib import Path

def validate_json_files(directory):
    if not Path(directory).exists():
        print(f"[INFO] Directory '{directory}' does not exist. Skipping.")
        return True

    for file in Path(directory).glob("*.json"):
        try:
            with open(file, "r") as f:
                json.load(f)
            print(f"[OK] Valid JSON: {file}")
        except Exception as e:
            print(f"[ERROR] Invalid JSON: {file} -> {e}")
            return False

    return True


if __name__ == "__main__":
    target_dir = sys.argv[1] if len(sys.argv) > 1 else "scenarios"
    success = validate_json_files(target_dir)

    if not success:
        sys.exit(1)
