from __future__ import annotations

import os

from dotenv import load_dotenv

from redis_store import RedisStore

load_dotenv()


def env_true(name: str) -> bool:
    return os.getenv(name, "false").strip().lower() in {"1", "true", "yes", "on"}


def main() -> None:
    store = RedisStore()
    store.ping()

    result = store.initialize_fixed_data()
    print(f"[FIXED CONFIG] status={result['status']} version={result['version']}")
    print(f"[FIXED POINTS] {', '.join(result['fixed_points'])}")
    print(f"[SCAN CASES] {', '.join(result['scan_cases'])}")

    if env_true("SEED_DEMO_OBJECTS"):
        store.save_object_record(
            record_name="cup",
            replace=True,
            data={
                "class_name": "cup",
                "coordinate": [420.5, -135.2, 85.0],
                "confidence": 0.91,
                "material": "plastic",
                "contains_liquid": True,
                "grasp": {"type": "side", "width_mm": 60.0},
            },
        )
        store.save_object_record(
            record_name="toothbrush",
            replace=True,
            data={
                "label": "toothbrush",
                "pose": {
                    "frame": "base_link",
                    "x": 510.2,
                    "y": 80.4,
                    "z": 42.0,
                },
                "color": "blue",
                "hygiene_status": "clean",
            },
        )
        print("[DEMO] free-form object data created")


if __name__ == "__main__":
    main()
