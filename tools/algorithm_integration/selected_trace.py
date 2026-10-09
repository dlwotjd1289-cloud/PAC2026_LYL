"""Audit-only snapshots. Compute Jaesung metrics after the runtime has stopped.

No imports from this module belong in a policy, placer, or runtime node.
The old open-pallet-edge-as-wall bug stays corrected in _jaesung_metrics.
"""
from copy import deepcopy


def capture_selected(core, command, decision_s):
    record = {"record_id": f"{command.state_version}:{command.box_id}:{command.action}",
              "action": command.action, "timing": {"decision_s": decision_s},
              "execution": {}, "provenance": {
                  "role": "post_decision_audit_only", "safety_verified": False,
                  "decision_source": "runtime_wall_clock",
                  "robot_source": "kinematic_model_not_physical_measurement",
                  "placement_scope": "one_selected_step_not_full_order_feasibility"}}
    if command.candidate is None:
        return {"record": record}
    state = core.sm.snapshot()
    box = core.sm.tracked[command.box_id]
    world = core.decider.snapshot_world(state, current_box_id=core.sm.current_id(),
        buffer_slots=core.sm.buffer_slots(), buffer_age=core.sm.buffer_age(),
        visible_boxes=core.preview)
    from pac_highlevel.merged_placement import future_context_from_world
    future = future_context_from_world(world, box)
    record["timing"]["robot_estimate_s"] = command.robot.get("cycle_time_s")
    record["execution"].update(robot_ok=True,
        clearance_m=command.robot.get("arm_clearance_m"))
    return {"record": record, "state": state, "box": box,
            "candidate": command.candidate, "context": deepcopy(core.sm.context()),
            "config": deepcopy(core.decider.cand_config), "future": future}


def evaluate_snapshot(snapshot):
    """Offline geometry replay of the selected candidate only; no IK or search."""
    from evaluate_five_metrics import evaluate_record
    record = deepcopy(snapshot["record"])
    if "candidate" in snapshot:
        from pac_candidates import CandidateBackend
        from pac_candidates.geometry import rotated_dims
        from pac_highlevel.merged_placement import (
            MergedConfig, _jaesung_grid, _jaesung_metrics)
        state, box, cand = (snapshot[k] for k in ("state", "box", "candidate"))
        verdict = CandidateBackend(snapshot["context"], snapshot["config"]).validate_constraints(box, cand, state)
        record["provenance"]["static_replay_success"] = verdict.success
        metrics = verdict.details.get("metrics", {})
        # Missing evidence is not replaced with a perfect score.
        required = ("support_ratio", "on_floor", "max_load_ratio")
        if verdict.success and all(k in metrics for k in required):
            pallet, p = state.pallet, cand.target_pose
            size = pallet.size
            dx, dy, dz = rotated_dims(box.size, p.yaw)
            heavy = snapshot["future"].heavier_share(box.weight_kg)
            detail = _jaesung_metrics(cand, box, metrics, _jaesung_grid(pallet), size, MergedConfig(), heavy)
            record["geometry_features"] = dict(z=p.z/size.z, top=(p.z+dz)/size.z,
                corner=(p.x+p.y)/(size.x+size.y), **{k: detail[k] for k in ("rough", "void", "block", "floor")})
            record["stability_components"] = {k: detail[k] for k in ("support", "interlock", "tilt", "side")}
            cog = metrics.get("pallet_cog_after_m")
            record["load"] = {"max_load_ratio": metrics["max_load_ratio"],
                              "cog_xyz_m": list(cog) if cog is not None else None}
            record["packing_fraction"] = (sum(b.size.x*b.size.y*b.size.z for b in pallet.boxes)+dx*dy*dz)/(size.x*size.y*size.z)
            record["provenance"].update(geometry="selected_planned_pose_model_not_measured_layout",
                edge_model="open_pallet_edges_are_not_walls", coordinate_frame=p.frame_id,
                acceleration_g=detail["critical_acceleration_g"],
                acceleration_scope="single_box_proxy_not_stack_transport_test",
                future_range_sources=list(snapshot["future"].range_sources))
        else:
            record.setdefault("missing_reasons", {})["geometry_features"] = "static_replay_missing_or_rejected"
    return {"input": record, "evaluation": evaluate_record(record)}
