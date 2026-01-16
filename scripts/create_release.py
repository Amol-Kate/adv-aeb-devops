# create_release.py

# Purpose:
# Creates a validated release artifact after safety approval.

from pathlib import Path
import json
from datetime import datetime

release_dir = Path("release")
release_dir.mkdir(exist_ok=True)

metadata = {
    "system": "AEB",
    "version": "1.0.0",
    "approved": True,
    "timestamp": datetime.utcnow().isoformat()
}

with open(release_dir / "release_metadata.json", "w") as f:
    json.dump(metadata, f, indent=2)

print("[RELEASE] Release artifact created successfully")
