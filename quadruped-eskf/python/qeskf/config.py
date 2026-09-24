"""Single source of truth for the estimator configuration used by every script.

IMU terms come from the datasheet (imu_model.ImuSpec) and are never tuned.
Contact-model terms are chosen by scripts/tune.py on the TUNING trajectory and
written to results/tuned_params.json; the headline run only reads that file.
"""
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path

from .eskf import EskfParams
from .imu_model import ImuSpec

ROOT = Path(__file__).resolve().parents[2]
TUNED = ROOT / "results" / "tuned_params.json"


@dataclass
class ContactPolicy:
    use_schedule: bool = True
    liftoff_advance_ms: float = 0.0
    delay_ms: float = 0.0


def load_tuned():
    """(EskfParams, ContactPolicy) from results/tuned_params.json."""
    d = json.loads(TUNED.read_text())
    prm = EskfParams.from_imu_spec(ImuSpec(), **d["eskf"])
    return prm, ContactPolicy(**d["contact"])


def save_tuned(eskf_overrides: dict, policy: ContactPolicy, provenance: dict):
    TUNED.parent.mkdir(parents=True, exist_ok=True)
    TUNED.write_text(json.dumps(dict(eskf=eskf_overrides, contact=asdict(policy), how_chosen=provenance),
                                indent=2))


def with_overrides(prm: EskfParams, **kw):
    return replace(prm, **kw)
