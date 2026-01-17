# safety_gate.py

# Purpose:
# Evaluates safety KPIs and decides whether release is allowed.

import json
import sys
from pathlib import Path

# KPI_FILE = Path("sim_results/kpis.json")
KPI_FILE = "artifacts/simulation/kpi_results.json"

# Safety thresholds (can be ASIL-tuned later)
MIN_TTC_THRESHOLD = 1.5      # seconds
MAX_JERK_THRESHOLD = 5.0     # m/s^3
MIN_COVERAGE = 0.9           # 90%

if not KPI_FILE.exists():
    print("[SAFETY] KPI file not found. Blocking release.")
    sys.exit(1)

with open(KPI_FILE, "r") as f:
    kpis = json.load(f)

print("[SAFETY] Evaluating KPIs:", kpis)

if kpis["collision"]:
    print("[FAIL] Collision detected")
    sys.exit(1)

if kpis["min_ttc"] < MIN_TTC_THRESHOLD:
    print("[FAIL] TTC below threshold")
    sys.exit(1)

if kpis["max_jerk"] > MAX_JERK_THRESHOLD:
    print("[FAIL] Excessive jerk detected")
    sys.exit(1)

if kpis["scenario_coverage"] < MIN_COVERAGE:
    print("[FAIL] Scenario coverage insufficient")
    sys.exit(1)

print("[PASS] Safety gate passed")
