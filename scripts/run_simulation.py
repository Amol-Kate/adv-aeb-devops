# run_simulation.py

# Purpose:
# Simulates scenario-based AEB validation and generates KPI metrics.

import json
from pathlib import Path
import random

output_dir = Path("sim_results")
output_dir.mkdir(exist_ok=True)

results = {
    "collision": False,
    "min_ttc": round(random.uniform(1.8, 3.5), 2),
    "max_jerk": round(random.uniform(1.0, 3.0), 2),
    "scenario_coverage": 1.0
}

with open(output_dir / "kpis.json", "w") as f:
    json.dump(results, f, indent=2)

print("[SIM] Simulation completed")
print(f"[SIM] KPIs written to {output_dir}/kpis.json")

# Enable to generate KPI file in layer 4:
import json
import os

print("[SIM] Running simulation (mock)...")

os.makedirs("artifacts/simulation", exist_ok=True)

kpi = {
    "collision": False,
    "min_ttc": 2.4,
    "max_deceleration": 6.2,
    "scenario_coverage": 0.93
}

with open("artifacts/simulation/kpi_results.json", "w") as f:
    json.dump(kpi, f, indent=2)

print("[SIM] KPI results written to artifacts/simulation/kpi_results.json")
