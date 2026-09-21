"""Filter raw clips with more than ten seconds of no hand-object contact."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
from typing import Any

import h5py
import numpy as np

NO_CONTACT_LIMIT_NS = 10_000_000_000


class ContactFilterError(ValueError):
    pass


def filter_contact_mask(
    timestamps_ns: Any,
    contact_mask: Any,
    *,
    threshold_ns: int = NO_CONTACT_LIMIT_NS,
) -> tuple[np.ndarray, list[dict[str, int]]]:
    timestamps = np.asarray(timestamps_ns, dtype=np.uint64)
    contacts = np.asarray(contact_mask, dtype=np.bool_)
    if timestamps.ndim != 1 or contacts.shape != timestamps.shape:
        raise ContactFilterError("timestamps and contact mask must have the same 1-D shape")
    if timestamps.size and np.any(np.diff(timestamps.astype(np.int64)) <= 0):
        raise ContactFilterError("contact timestamps must be strictly increasing")
    keep = np.ones(timestamps.shape, dtype=np.bool_)
    audit: list[dict[str, int]] = []
    start: int | None = None
    for index, active in enumerate(contacts):
        if not active and start is None:
            start = index
        if active and start is not None:
            end = index
            if timestamps[end - 1] - timestamps[start] > threshold_ns:
                keep[start:end] = False
                audit.append({
                    "start_ns": int(timestamps[start]),
                    "end_ns": int(timestamps[end - 1]),
                    "samples": end - start,
                })
            start = None
    if start is not None and timestamps.size:
        end = timestamps.size
        if timestamps[end - 1] - timestamps[start] > threshold_ns:
            keep[start:end] = False
            audit.append({
                "start_ns": int(timestamps[start]),
                "end_ns": int(timestamps[end - 1]),
                "samples": end - start,
            })
    return keep, audit


def filter_episode(input_h5: str | Path, output_h5: str | Path) -> dict[str, Any]:
    input_h5, output_h5 = Path(input_h5), Path(output_h5)
    with h5py.File(input_h5, "r") as source:
        if "contacts/hand_object" not in source:
            raise ContactFilterError("raw episode has no contacts/hand_object dataset")
        timestamps = source["timestamps/robot_ns"][:]
        mask = source["contacts/hand_object"][:]
        keep, audit = filter_contact_mask(timestamps, mask)
        output_h5.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix=f".{output_h5.name}.staging-", dir=output_h5.parent)
        import os
        os.close(fd)
        staging = Path(name)
        staging.unlink()
        try:
            with h5py.File(staging, "w") as target:
                for key, value in source.attrs.items():
                    target.attrs[key] = value

                def copy_item(name: str, value: h5py.Dataset | h5py.Group) -> None:
                    if isinstance(value, h5py.Group):
                        target.require_group(name)
                        return
                    destination = target
                    parts = name.split("/")
                    for part in parts[:-1]:
                        destination = destination.require_group(part)
                    data = value[:]
                    if name in {
                        "timestamps/robot_ns", "observations/qpos", "observations/qvel",
                        "actions/qpos_target", "validity/arm_mask", "validity/hand_mask",
                        "timestamps/contacts_ns", "contacts/json", "contacts/hand_object",
                    }:
                        data = data[keep]
                    kwargs = {"compression": "gzip"} if getattr(data, "ndim", 0) > 0 else {}
                    destination.create_dataset(parts[-1], data=data, **kwargs)

                source.visititems(copy_item)
                target.require_group("filter").create_dataset(
                    "audit_json",
                    data=np.asarray(json.dumps(audit), dtype=h5py.string_dtype()),
                )
                target.attrs["filtered_no_contact_threshold_ns"] = NO_CONTACT_LIMIT_NS
                target.flush()
            output_h5.unlink(missing_ok=True)
            staging.replace(output_h5)
        except Exception:
            staging.unlink(missing_ok=True)
            raise
    return {"input_samples": int(keep.size), "kept_samples": int(np.count_nonzero(keep)), "removed_spans": audit}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_h5", type=Path)
    parser.add_argument("output_h5", type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(filter_episode(args.input_h5, args.output_h5), sort_keys=True))
    return 0


__all__ = ["ContactFilterError", "filter_contact_mask", "filter_episode", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
