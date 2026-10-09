"""Evaluate a chosen placement trace with preserved Jaesung arithmetic only.

No candidate generation, selection, IK, hard mask, execution or optimization is
performed. Inputs are features recorded AFTER the team's placement selection.

Input contract:
  geometry_features: z, top, rough, void, corner, block, floor (normalized)
  packing_fraction: observed occupied-volume fraction, reported separately
  stability_components: support, interlock, tilt, side (each in [0, 1])
  load: max_load_ratio (>=0), optional cog_xyz_m=[x,y,z]
  timing: decision_s (measured), robot_estimate_s (model estimate)
  execution: robot_ok, clearance_m, result_ok
  missing_reasons: optional mapping from dotted input path to explanation
  provenance: optional JSON evidence metadata, echoed without verification

Missing values produce null plus a reason. Invalid supplied values raise.
This module does not reconstruct geometry or assume a missing metric is safe.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path


EFF_WEIGHTS = {"z": 2.0, "top": 2.0, "rough": 2.0, "void": 1.5,
               "corner": 4.0, "block": 1.5, "floor": .5}
STABILITY_WEIGHTS = {"support": .20, "interlock": .25, "tilt": .40, "side": .15}


def phi(value, target, low, high):
    return low * min(value, target) + high * max(0.0, value-target)


def _metric(value, reason=None, **metadata):
    return {"value": value, "reason": reason, **metadata}


def _missing(record, path):
    reasons = record.get("missing_reasons") or {}
    if not isinstance(reasons, dict):
        raise ValueError("missing_reasons must be an object")
    reason = reasons.get(path)
    if reason is not None and (not isinstance(reason, str) or not reason.strip()):
        raise ValueError(f"{path}: missing reason must be nonempty text")
    return _metric(None, reason or f"Not recorded: {path}")


def _value(record, path):
    current=record
    for key in path.split("."):
        if current is None:
            return None
        if not isinstance(current,dict):
            raise ValueError(f"{path}: parent must be an object")
        current=current.get(key)
    return current


def _number(record, path, minimum=None, maximum=None):
    value=_value(record,path)
    if value is None:
        return _missing(record,path)
    if isinstance(value,bool) or not isinstance(value,(int,float)):
        raise ValueError(f"{path}: finite number required")
    try:
        valid=math.isfinite(value)
    except OverflowError:
        valid=False
    if not valid or (minimum is not None and value<minimum) or (maximum is not None and value>maximum):
        raise ValueError(f"{path}: invalid finite/range value")
    return _metric(value)


def _boolean(record,path):
    value=_value(record,path)
    if value is None:return _missing(record,path)
    if type(value) is not bool:raise ValueError(f"{path}: boolean required")
    return _metric(value)


def _derived(inputs, compute, formula):
    absent=[key for key,value in inputs.items() if value["value"] is None]
    if absent:
        return _metric(None,"Missing required inputs: "+", ".join(absent),formula=formula)
    value=compute({key:metric["value"] for key,metric in inputs.items()})
    if not math.isfinite(value):raise ValueError("Derived value exceeds finite arithmetic range")
    return _metric(value,formula=formula)


def evaluate_record(record):
    """Pure read-only JSON evaluation. No placement behavior can be changed."""
    if not isinstance(record,dict):raise ValueError("record must be a JSON object")
    try:json.dumps(record,allow_nan=False)
    except (TypeError,ValueError,OverflowError) as error:raise ValueError("record must contain finite JSON values") from error
    geometry={name:_number(record,"geometry_features."+name,minimum=0) for name in EFF_WEIGHTS}
    efficiency=_derived(geometry,lambda values:sum(EFF_WEIGHTS[k]*v for k,v in values.items()),
                        "2*z + 2*top + 2*rough + 1.5*void + 4*corner + 1.5*block + 0.5*floor")
    components={name:_number(record,"stability_components."+name,0,1) for name in STABILITY_WEIGHTS}
    stability=_derived(components,lambda values:sum(STABILITY_WEIGHTS[k]*v for k,v in values.items()),
                       "0.20*support + 0.25*interlock + 0.40*tilt + 0.15*side")
    phi_s=_derived({"S":stability},lambda values:phi(values["S"],.75,20.,2.),
                   "20*min(S,0.75) + 2*max(0,S-0.75)")
    ratio=_number(record,"load.max_load_ratio",minimum=0)
    margin=_derived({"max_load_ratio":ratio},lambda values:min(1.,max(0.,1.-values["max_load_ratio"])),
                    "clip(1-max_load_ratio,0,1)")
    phi_l=_derived({"L":margin},lambda values:phi(values["L"],.5,8.,.5),
                   "8*min(L,0.5) + 0.5*max(0,L-0.5)")
    cog=_value(record,"load.cog_xyz_m")
    if cog is None:
        cog_metric=_missing(record,"load.cog_xyz_m")
    else:
        if not isinstance(cog,list) or len(cog)!=3:
            raise ValueError("load.cog_xyz_m must be a three-number JSON array")
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in cog):
            raise ValueError("load.cog_xyz_m must contain finite numbers")
        cog_metric=_metric(list(cog),unit="m",interpretation="Coordinate only; not part of original L/phi_L")
    legacy=_derived({"efficiency_cost":efficiency,"phi_S":phi_s,"phi_L":phi_l},
                    lambda values:values["efficiency_cost"]-values["phi_S"]-values["phi_L"],
                    "efficiency_cost - phi_S - phi_L")
    legacy.update(lower_is_better=True,is_five_category_total=False,
                  interpretation="Original candidate formula only; excludes timing, CoG and execution outcomes")
    result={
        "schema_version":"five-metric-evaluation-1.0","record_id":deepcopy(record.get("record_id")),
        "evaluation_only":True,
        "space":{"geometry_features":geometry,"weights":dict(EFF_WEIGHTS),"legacy_efficiency_cost":efficiency,
                 "packing_fraction":_number(record,"packing_fraction",minimum=0),
                 "packing_note":"Supplied volume fraction, independent of original efficiency cost. Values above one are not clipped and require geometry review."},
        "time":{"decision_s":dict(_number(record,"timing.decision_s",minimum=0),basis="measured wall seconds supplied by caller"),
                "robot_estimate_s":dict(_number(record,"timing.robot_estimate_s",minimum=0),basis="existing robot time estimate, not measured execution"),
                "legacy_time_score":_metric(None,"No time term in this original placement formula; no score invented")},
        "load":{"max_load_ratio":ratio,"L":margin,"phi_L":phi_l,"cog_xyz_m":cog_metric},
        "stability":{"components":components,"weights":dict(STABILITY_WEIGHTS),"S":stability,"phi_S":phi_s},
        "execution_feasibility":{"robot_ok":_boolean(record,"execution.robot_ok"),
            "clearance_m":_number(record,"execution.clearance_m"),"result_ok":_boolean(record,"execution.result_ok"),
            "probability_score":_metric(None,"Boolean check/outcome and clearance only; no probability inferred")},
        "legacy_cost":legacy,"provenance":deepcopy(record.get("provenance",{})),
        "assumptions":[
            "Original efficiency weights, S weights and nonlinear targets/slopes are fixed design choices, not newly optimized or measured.",
            "Input geometry and stability components are supplied trace features; this evaluator does not recreate or verify their physical model.",
            "tilt is the supplied normalized original soft-model component; it is not a transport test or a 0.3g hard gate.",
            "CoG and signed clearance are reported separately; coordinate frame and derivation should be supplied in provenance.",
            "An execution result, robot feasibility check and estimated cycle time are distinct facts.",
        ],
        "performed":{"selection":False,"ik":False,"hard_mask":False,"simulation":False},
    }
    json.dumps(result,allow_nan=False)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,required=True)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    try:
        data=json.loads(args.input.read_text(encoding="utf-8"))
        result=[evaluate_record(row) for row in data] if isinstance(data,list) else evaluate_record(data)
        text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n"
    except (OSError,ValueError,OverflowError) as error:parser.error(str(error))
    if args.output:args.output.write_text(text,encoding="utf-8")
    else:print(text,end="")


if __name__=="__main__":main()
