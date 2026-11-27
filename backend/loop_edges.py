"""Geometric proposals, disjoint held-out areas and independently checked GICP factors."""
from functools import lru_cache
import math

import numpy as np
import small_gicp
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from frame_io import read_pcd, rigid_inverse, transform, voxel


def rotation_deg(T):
    return float(np.rad2deg(Rotation.from_matrix(T[:3, :3]).magnitude()))


def select_pairs(poses, times, p0, maximum, radius, min_seconds, fixed_pairs=None):
    xyz = poses[:, :3, 3]
    track = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
    tree = cKDTree(xyz)
    proposals = []
    last = 0
    for j in range(1, len(times) - 1):
        if track[j] - track[last] < 1.8 and times[j] - times[last] < 15:
            continue
        last = j
        ids = [i for i in tree.query_ball_point(xyz[j], radius)
               if 0 < i < j and times[j] - times[i] >= min_seconds and track[j] - track[i] >= 18.]
        representatives = []
        for i in sorted(ids, key=lambda i: (np.linalg.norm(xyz[i] - xyz[j]), i)):
            if all(abs(times[i] - times[k]) >= min_seconds for k in representatives):
                representatives.append(i)
                proposals.append((i, j))
                if len(representatives) >= 2:
                    break
    # Choose before reading any cloud or estimating any constraint. Preserve both
    # visits, reject their neighboring frames and mask their world-space cylinders.
    hold = list(fixed_pairs or [])
    centers = [xyz[k,:2] for pair in hold for k in pair]
    anchor = xyz[p0[0] if p0 else 0, :2]
    for i, j in sorted(proposals if fixed_pairs is None else [], key=lambda ij: (-min(track[ij[1]]-track[ij[0]], 120),
                                                   np.linalg.norm(xyz[ij[1]]-xyz[ij[0]]))):
        loc = xyz[[i, j], :2]
        if np.min(np.linalg.norm(loc - anchor, axis=1)) < 12:
            continue
        if any(np.min(np.linalg.norm(loc - c, axis=1)) < 12 for c in centers):
            continue
        hold.append((i, j))
        centers.extend(loc)
        if len(hold) >= 5:
            break
    centers = np.asarray(centers, dtype=float).reshape(-1, 2)
    excluded = {k for ij in hold for i in ij for k in range(max(0, i-2), min(len(times), i+3))}
    def eligible(pair):
        if any(k in excluded for k in pair):
            return False
        return all(np.min(np.linalg.norm(xyz[list(pair), :2]-c, axis=1)) > 4 for c in centers)
    remaining = [p for p in proposals if eligible(p)]
    # Group by current visit before deterministic temporal stratification. Evaluate
    # both alternative historical visits for a selected query.
    group = {}
    for i, j in remaining:
        group.setdefault(j, []).append((i, j))
    queries = list(group)
    if len(queries) > maximum:
        queries = [queries[k] for k in np.linspace(0, len(queries)-1, maximum, dtype=int)]
    return hold, centers, excluded, [p for j in queries for p in group[j]], len(proposals)


def p0_pair(times, poses, windows):
    if windows is None:
        return None, []
    chosen, reports = [], []
    for lo, hi in np.reshape(windows, (2, 2)):
        ids = np.flatnonzero((times >= lo) & (times <= hi))
        if len(ids) < 2:
            raise ValueError("not enough keyframes in tape window")
        xyz = poses[ids, :3, 3]
        center = np.median(xyz, axis=0)
        # Choose representative near the centre of position distribution, with
        # neighbours in the window when possible.
        inner = ids[1:-1] if len(ids) >= 3 else ids
        k = int(inner[np.argmin(np.linalg.norm(poses[inner, :3, 3]-center, axis=1))])
        chosen.append(k)
        dis = np.linalg.norm(xyz - center, axis=1)
        reports.append({"lo_s": float(lo), "hi_s": float(hi), "n": len(ids), "selected_index": k,
                        "radius_p90_m": float(np.quantile(dis, .9)), "span_s": float(times[ids[-1]]-times[ids[0]])})
    if chosen[1] <= chosen[0]:
        raise ValueError("tape windows must be distinct and chronological")
    return tuple(chosen), reports


class EdgeBuilder:
    def __init__(self, frames, poses, centers, excluded):
        self.frames, self.poses = frames, poses
        self.centers, self.excluded = centers, excluded
        self.masked_count = 0

    @lru_cache(maxsize=64)
    def scan(self, k):
        xyz, _ = read_pcd(self.frames[k].cloud)
        return voxel(xyz, .12)

    def samples(self, index, ids):
        old = rigid_inverse(self.poses[index])
        pieces = []
        for k in ids:
            if k < 0 or k >= len(self.frames) or k in self.excluded:
                continue
            world = transform(self.scan(k), self.poses[k])
            mask = np.ones(len(world), bool)
            for c in self.centers:
                mask &= np.sum(np.square(world[:, :2]-c), axis=1) > 4.**2
            self.masked_count += int(np.count_nonzero(~mask))
            pieces.append(transform(world[mask], old))
        return voxel(np.vstack(pieces), .16) if pieces else np.empty((0, 3))

    def fit(self, target, source, guess):
        return small_gicp.align(np.ascontiguousarray(target, dtype=float), np.ascontiguousarray(source, dtype=float),
                                init_T_target_source=guess, registration_type="GICP", downsampling_resolution=.18,
                                max_correspondence_distance=.9, max_iterations=40, num_threads=2)

    def estimate(self, i, j, kind):
        base = rigid_inverse(self.poses[i]) @ self.poses[j]
        row = {"target_index": i, "source_index": j, "target_id": self.frames[i].id, "source_id": self.frames[j].id,
               "kind": kind, "T_prior_target_source": base.tolist(), "accepted": False, "reasons": []}
        # Primary and corroborating fits use completely disjoint keyframe scans.
        target = self.samples(i, (i-2, i, i+2))
        source = self.samples(j, (j-2, j, j+2))
        tn = self.samples(i, (i-1, i+1))
        sn = self.samples(j, (j-1, j+1))
        row["submap_sizes"] = [len(c) for c in (target, source, tn, sn)]
        if min(row["submap_sizes"]) < 80:
            row["reasons"].append("insufficient_unmasked_geometry")
            return row
        guesses = [base]
        if kind == "tape":
            same_area = base.copy()
            same_area[:3, 3] = 0.
            guesses.extend((same_area, np.eye(4)))
        tree = cKDTree(target)
        def quality(T):
            src = transform(source, T)
            d, near = tree.query(src)
            rev = cKDTree(src).query(target)[0]
            ok = d < .32
            cover = float(np.mean(ok))
            reverse = float(np.mean(rev < .32))
            rms = float(np.sqrt(np.mean(d[ok]**2))) if np.any(ok) else 10.
            return min(cover, reverse), rms, cover, reverse, near, ok
        fits = []
        for guess in guesses:
            sol = self.fit(target, source, guess)
            if np.isfinite(sol.T_target_source).all():
                fit = quality(sol.T_target_source)
                fits.append((sol, fit))
        if not fits:
            row["reasons"].append("nonfinite_registration")
            return row
        sol, q = max(fits, key=lambda x: (x[0].converged, x[1][0]-.7*x[1][1]))
        T = np.array(sol.T_target_source)
        # Local plane-normal moment is a direct test of translation observability;
        # unlike NN distance it responds to a missing corridor direction.
        ids = np.unique(q[4][q[5]])
        ids = ids[::max(1, math.ceil(len(ids)/1600))]
        eig = np.zeros(3)
        eig6 = np.zeros(6)
        weak = None
        moment_source = np.zeros((6,6))
        lever_radius = .25
        sectors = 1
        if len(ids) >= 20:
            patch = target[tree.query(target[ids], k=12)[1]]
            centered = patch - np.mean(patch, axis=1, keepdims=True)
            values, vec = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
            planar = values[:, 0] < .20*np.maximum(values[:, 1], 1e-10)
            normals = vec[planar, :, 0]
            if len(normals) >= 20:
                eig, axes = np.linalg.eigh(normals.T @ normals / len(normals))
                weak = (self.poses[i][:3,:3] @ axes[:,0]).tolist()
                centers = target[ids][planar]
                centers = centers-centers.mean(axis=0)
                lever_radius = max(.25,np.sqrt(np.mean(np.sum(centers**2,axis=1))))
                # Rotate normalized information from the target axes into the
                # source-body right tangent of BetweenFactorPose3.
                rot = T[:3,:3]
                jac = np.c_[np.cross(centers/lever_radius,normals)@rot,normals@rot]
                moment_source = jac.T@jac/len(normals)
                eig6 = np.linalg.eigvalsh(moment_source)
                sectors = len(np.unique(centers>=0,axis=0))
        neighbor = self.fit(tn, sn, T)
        difference = rigid_inverse(T) @ neighbor.T_target_source
        change = rigid_inverse(base) @ T
        row.update(T_target_source=T.tolist(), converged=bool(sol.converged), iterations=int(sol.iterations),
                   neighbor_converged=bool(neighbor.converged), neighbor_inliers=int(neighbor.num_inliers),
                   coverage_source=q[2], coverage_target=q[3], rms_inlier_m=q[1],
                   normal_moment_eigenvalues=eig.tolist(), weakest_translation_axis_map_gravity=weak,
                   normalized_point_plane_eigenvalues=eig6.tolist(), small_gicp_hessian_eigenvalues=np.linalg.eigvalsh(sol.H).tolist(),
                   normalized_point_plane_moment_source=moment_source.tolist(), matched_lever_radius_m=float(lever_radius),
                   occupied_centered_octants=sectors, T_primary_inverse_neighbor=difference.tolist(),
                   neighbor_diff_m=float(np.linalg.norm(difference[:3, 3])), neighbor_diff_deg=rotation_deg(difference),
                   correction_m=float(np.linalg.norm(change[:3, 3])), correction_deg=rotation_deg(change),
                   registration_seeds=len(guesses), inlier_correspondences=int(sol.num_inliers))
        checks = [(not sol.converged, "not_converged"), (q[0]<.38, "low_bidirectional_coverage"),
                  (q[1]>.19, "high_residual"), (eig[0]<.035, "weak_translation_direction"),
                  (eig6[0]<.0015,"weak_6dof_direction"),
                  (not neighbor.converged or neighbor.num_inliers < 60 or row["neighbor_diff_m"]>.30 or
                   row["neighbor_diff_deg"]>5., "inconsistent_adjacent_scans"),
                  (row["correction_m"]>(5.0 if kind == "tape" else 2.5) or row["correction_deg"]>35,
                   "excessive_prior_disagreement")]
        row["reasons"] = [label for failed, label in checks if failed]
        row["accepted"] = not row["reasons"]
        return row


def screen(edges, poses, times):
    accepted = []
    # For one present visit keep at most one measurement of the same scans,
    # even if two historical submaps support compatible map poses.
    ordered = sorted(edges, key=lambda e:(e["kind"]!="tape",e["source_index"],
                     -min(e.get("coverage_source",0),e.get("coverage_target",0))+.7*e.get("rms_inlier_m",10)))
    for edge in ordered:
        if not edge["accepted"]:
            continue
        i, j = edge["target_index"], edge["source_index"]
        if edge["kind"] == "automatic":
            for other in edges:
                if other is edge or other["source_index"] != j or other.get("reasons") or not other.get("T_target_source"):
                    continue
                # Incompatible map-frame predictions with comparable match quality
                # leave the identity of the place unresolved.
                a = poses[i] @ np.array(edge["T_target_source"])
                b = poses[other["target_index"]] @ np.array(other["T_target_source"])
                if (np.linalg.norm(a[:3, 3]-b[:3, 3]) > .75 and
                    abs(min(edge["coverage_source"], edge["coverage_target"]) -
                        min(other["coverage_source"], other["coverage_target"])) < .12):
                    edge["accepted"] = other["accepted"] = False
                    edge["reasons"].append("ambiguous_competing_history")
                    other["reasons"].append("ambiguous_competing_history")
                    break
        if edge["accepted"] and any(j==k["source_index"] or (abs(times[j]-times[k["source_index"]]) < 20 and
                                    abs(times[i]-times[k["target_index"]]) < 20) for k in accepted if k["accepted"]):
            edge["accepted"] = False
            edge["reasons"].append("overlapping_accepted_constraint")
        if edge["accepted"]:
            accepted.append(edge)
    return [e for e in accepted if e["accepted"]]
