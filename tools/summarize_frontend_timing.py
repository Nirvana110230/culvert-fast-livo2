"""Summarize FAST_LIVO_KEYFRAMES diagnostics emitted by the instrumented mapper."""
import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def rows(directory, filename):
    with (directory / filename).open(encoding="utf-8", newline="") as source:
        return list(csv.DictReader(source))


def total(records, field):
    return sum(int(row[field]) for row in records)


def distribution(values):
    values = sorted(values)
    if not values:
        return {"n": 0}
    return {"n": len(values), "min": values[0], "p50": statistics.median(values),
            "p95_nearest_rank": values[math.ceil(0.95 * len(values)) - 1], "max": values[-1]}


def aggregate_planes(records):
    grouped = defaultdict(list)
    for row in records:
        grouped[row["class"]].append(row)
    result = {}
    for name, group in grouped.items():
        n = total(group, "count")
        denom = total(grouped["all"], "count")
        result[name] = {"count": n, "fraction_of_all": n/denom if denom else None}
        if n:
            result[name].update({
                "mean_abs_m": sum(float(r["abs_sum_m"]) for r in group) / n,
                "rms_m": math.sqrt(sum(float(r["sq_sum_m"]) for r in group) / n),
                "max_abs_m": max(float(r["max_abs_m"]) for r in group),
                "per_nonempty_window_p95_abs_m": distribution(
                    float(r["p95_abs_m"]) for r in group if int(r["count"])),
                "mean_normal_outer_product_xx_xy_xz_yy_yz_zz": [
                    sum(float(r[f"n{ij}_sum"]) for r in group) / n
                    for ij in ("xx", "xy", "xz", "yy", "yz", "zz")]})
    return result


def normal_support(records, initial_up):
    # For an unweighted point-to-plane translation Jacobian row n^T, the
    # normalized information along u is sum((n dot u)^2)/N.
    x,y,z=initial_up
    values=[]; world_z=[]
    for r in records:
        n=int(r["count"])
        if r["class"]!="all" or not n:
            continue
        xx,xy,xz,yy,yz,zz=(float(r[f"n{ij}_sum"])/n for ij in ("xx","xy","xz","yy","yz","zz"))
        values.append(xx*x*x+2*xy*x*y+2*xz*x*z+yy*y*y+2*yz*y*z+zz*z*z)
        world_z.append(zz)
    return {"reference_unit_up_in_world":initial_up,
            "assumption":"B1 initial world=IMU frame and uav.gravity_align_en=false",
            "per_update_along_initialization_up":distribution(values),
            "per_update_along_world_z":distribution(world_z),
            "definition":"sum((n dot u)^2)/N over final point-to-plane correspondences; unweighted translation sensitivity, excludes lever arm, covariance, visual/IMU constraints"}


def summarize(directory):
    windows = rows(directory, "timing_windows.csv")
    scans = rows(directory, "preprocess_timing.csv")
    deskew = rows(directory, "deskew_timing.csv")
    imu = rows(directory, "imu_init_samples.csv")
    planes = rows(directory, "plane_constraints.csv")
    if not windows or not imu or not scans or not deskew or not planes:
        raise ValueError("incomplete frontend evidence: one or more tables are empty")
    cuts = [float(r["end_s"]) for r in windows]
    if any(b <= a for a, b in zip(cuts, cuts[1:])):
        raise ValueError("accepted image timestamps must be strictly increasing")
    spans = Counter()
    for scan in scans:
        if not int(scan["output_points"]):
            continue
        header = float(scan["scan_s"])
        first = header + float(scan["min_output_ms"]) * 0.001
        last = header + float(scan["max_output_ms"]) * 0.001
        # Strictly inside the retained scan span, separating the 1 us boundary band.
        count = max(0, bisect.bisect_left(cuts, last - 1e-6) - bisect.bisect_right(cuts, first + 1e-6))
        spans[count] += 1
    source, assigned = total(scans, "output_points"), total(windows, "current_points")
    popped = total(windows, "new_points")
    tail = int(windows[-1]["carry_out"])
    queued_tail = source - popped
    early = total(windows, "early_gt_1us")
    imu_t = [float(r["stamp_s"]) for r in imu]
    plane_times = {float(r["cut_s"]) for r in planes}
    sensors = {}
    for name, cols in {"acceleration_native_driver_units": ("ax", "ay", "az"), "angular_velocity_native_driver_units": ("gx", "gy", "gz")}.items():
        axes = [[float(r[key]) for r in imu] for key in cols]
        sensors[name] = {"axis_mean": list(map(statistics.mean, axes)),
                         "axis_population_std": list(map(statistics.pstdev, axes)),
                         "norm": distribution(math.sqrt(sum(x*x for x in sample)) for sample in zip(*axes))}
    mean_acc=sensors["acceleration_native_driver_units"]["axis_mean"]
    acc_norm=math.sqrt(sum(x*x for x in mean_acc))
    evidence_files = ["timing_windows.csv", "preprocess_timing.csv", "plane_constraints.csv", "imu_init_samples.csv", "deskew_timing.csv"]
    result = {
        "source": "live sync, preprocess, IMU_init, deskew and final-iteration plane match instrumentation",
        "input_file_sha256": {f:hashlib.sha256((directory/f).read_bytes()).hexdigest() for f in evidence_files},
        "criteria": {"clock":"source scan header plus effective float curvature, ms; before deskew",
                     "early":"acquisition strictly more than 1 microsecond after accepted end; exact and near-cut reported separately",
                     "surface_classes":"within 20 degrees of gravity axis and >0.20 m above/below LiDAR origin for ceiling/ground candidates; within 20 degrees of perpendicular for wall candidates",
                     "residual":"absolute signed point-to-plane distance from final accepted iteration, metres; each matched point counted once per update",
                     "initial_acceleration_units":"message-native Livox axes, no m/s^2 conversion by driver",
                     "normal_tensor":"unit plane-normal outer products in estimator world axes, invariant to normal sign"},
        "accepted_half_open_windows": len(cuts), "window_end_range_s": [cuts[0], cuts[-1]],
        "window_width_ms": distribution((float(r["end_s"])-float(r["start_s"]))*1000 for r in windows),
        "scan_count_by_number_of_interior_accepted_cuts": dict(sorted(spans.items())),
        "points": {"preprocess_output": source, "popped_from_source_queues": popped, "assigned": assigned,
                   "pending_next_window_at_end": tail, "callback_output_never_popped": queued_tail,
                   "last_accepted_window_observed_raw_queue_points": int(windows[-1]["queued_points"]),
                   "windows_with_count_mismatch": sum(r["identity_count_ok"] != "1" for r in windows),
                   "carry_continuity_errors": sum(int(b["carry_in"]) != int(a["carry_out"]) for a, b in zip(windows, windows[1:])),
                   "popped_minus_assigned_minus_pending": popped-assigned-tail,
                   "early_over_1us": early, "early_carried_over_1us": total(windows,"early_carried_gt_1us"),
                   "early_fraction_of_assigned": early / assigned if assigned else None,
                   "affected_window_count": sum(int(r["early_gt_1us"]) > 0 for r in windows),
                   "early_max_ms": max(float(r["early_max_ms"]) for r in windows),
                   "affected_window_maximum_ms": distribution(float(r["early_max_ms"]) for r in windows if int(r["early_gt_1us"])),
                   "worst_window_end_s": max(windows,key=lambda r:float(r["early_max_ms"]))["end_s"] if early else None,
                   "early_mean_ms": sum(float(r["early_sum_ms"]) for r in windows)/early if early else None,
                   "assigned_at_exact_end": total(windows, "assigned_exact_cut"),
                   "assigned_within_1us_of_end": total(windows, "assigned_within_1us_cut"),
                   "new_points_at_exact_end": total(windows,"new_exact_cut"),
                   "new_points_within_1us_of_end": total(windows,"new_within_1us_cut"),
                   "before_start_over_1us": total(windows, "before_start_gt_1us"),
                   "max_rebase_error_ms": max(float(r["max_rebase_error_ms"]) for r in windows)},
        "livox_preprocessing": {"messages": len(scans), "input_points": total(scans,"input_points"),
                                "smoothed_branch_input": total(scans,"smoothing_inputs"),
                                "changed": total(scans,"smoothing_changed"), "retained_changed": total(scans,"retained_changed"),
                                "max_abs_ms": max(float(r["smoothing_abs_max_ms"]) for r in scans),
                                "retained_max_abs_ms": max(float(r["retained_abs_max_ms"]) for r in scans),
                                "changed_mean_abs_ms": sum(float(r["smoothing_abs_sum_ms"]) for r in scans) /
                                    max(1,total(scans,"smoothing_changed"))},
        "deskew": {"windows": len(deskew), "input_points": total(deskew,"input_points"),
                   "transformed": total(deskew,"transformed_points"), "untransformed": total(deskew,"untransformed_points"),
                   "curvature_inversions": total(deskew,"time_inversions"),
                   "outside_propagation_interval_over_1us": total(deskew,"outside_interval_gt_1us")},
        "imu_initialization": {"samples": len(imu_t), "unique_stamps": len(set(imu_t)),
                               "input_groups": len({r["cut_s"] for r in imu}), "first_s": imu_t[0], "last_s": imu_t[-1],
                               "span_ms": (imu_t[-1]-imu_t[0])*1000, "configured_count": int(imu[-1]["limit"]),
                               "counter_after": int(imu[-1]["counter_after_group"]), **sensors},
        "plane_cuts_missing_from_window_table": len(plane_times-set(cuts)),
        "planes": aggregate_planes(planes),
        "plane_translation_sensitivity":normal_support(planes,[x/acc_norm for x in mean_acc])}
    for source in (directory/"input_counters.json",directory.parent/"Log"/"input_counters.json"):
        if source.is_file():
            result["mapper_final_input_counters"] = json.loads(source.read_text(encoding="utf-8"))
            break
    return result


def compare_windows(reference, candidate):
    base = {r["end_s"]:r for r in rows(reference,"timing_windows.csv")}
    trial = {r["end_s"]:r for r in rows(candidate,"timing_windows.csv")}
    common = sorted(base.keys() & trial.keys(), key=float)
    delta = [int(trial[k]["current_points"])-int(base[k]["current_points"]) for k in common]
    differences = sorted(zip(common,delta),key=lambda x:abs(x[1]),reverse=True)
    return {"reference_windows":len(base), "candidate_windows":len(trial), "same_stamps":len(common),
            "same_start_and_end":sum(base[k]["start_s"]==trial[k]["start_s"] for k in common),
            "count_changed_on_paired_stamps":sum(d!=0 for d in delta), "net_change_on_paired_stamps":sum(delta),
            "maximum_absolute_window_count_change":max(map(abs,delta),default=0),
            "assigned_returns_acquired_after_cut_over_1us_reference":sum(int(base[k]["early_gt_1us"]) for k in common),
            "assigned_returns_acquired_after_cut_over_1us_candidate":sum(int(trial[k]["early_gt_1us"]) for k in common),
            "ten_largest_window_count_changes":[{"cut_s":k,"reference":int(base[k]["current_points"]),
                "candidate":int(trial[k]["current_points"]),"candidate_minus_reference":d} for k,d in differences[:10]]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--compare", type=Path, help="reference diagnostic directory for a paired comparison at each accepted cut")
    args = parser.parse_args()
    summary = summarize(args.directory)
    if args.compare:
        summary["paired_window_comparison"]=compare_windows(args.compare,args.directory)
    result = json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)
    if args.out:
        args.out.write_text(result + "\n", encoding="utf-8")
    else:
        print(result)
