#!/usr/bin/env python3
"""Read a recorded bag; measure failure formation without starting ROS nodes.

Requires ROS message libraries and a separately reconstructed baseline tree.
Writes only analysis artifacts beneath --output. Never publishes messages.
"""
from __future__ import annotations

import argparse
import bisect
import dataclasses
import json
import math
from pathlib import Path
import sqlite3
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True, type=Path)
    ap.add_argument("--baseline-root", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    args = ap.parse_args()
    for pkg in ("competition_planning", "competition_control", "competition_safety"):
        sys.path.insert(0, str(args.baseline_root / "src" / pkg))
    import numpy as np
    import yaml
    import rclpy
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer
    from competition_planning.local_trajectory_planner import (
        LocalReplanConfig, LocalTrajectoryPlanner, occupied_grid_cell_centers,
        _crop_planning_map, _overlay_obstacles,
    )
    from competition_planning.occupancy_grid_planner import OccupancyGridMap
    from competition_planning.semantic_planner import PathPoint
    from competition_planning.hybrid_astar_planner import _Node

    args.output.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(f"file:{args.bag.resolve().as_posix()}?mode=ro", uri=True)
    topics = {i: (n, typ) for i, n, typ in db.execute("select id,name,type from topics")}
    wanted = {"/tf", "/tf_static", "/scan", "/avoidance/local_costmap",
              "/planning/local_trajectory", "/planning/local_replan_status",
              "/control/status", "/control/tracking_error", "/control/body_cmd",
              "/cmd_vel_safe", "/cmd_vel", "/odom", "/safety/event",
              "/mission/route_enable", "/avoidance/proximity_status"}
    types = {i: get_message(typ) for i, (n, typ) in topics.items() if n in wanted}
    start = db.execute("select min(timestamp) from messages").fetchone()[0]
    t0 = start / 1e9
    stamp = lambda h: h.sec + h.nanosec / 1e9
    wrap = lambda a: math.atan2(math.sin(a), math.cos(a))

    def yaw(q):
        return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    rclpy.init()
    full = Buffer(cache_time=Duration(seconds=1000))
    online = Buffer(cache_time=Duration(seconds=1000))
    tf_ids = [i for i, (n, _) in topics.items() if n in {"/tf", "/tf_static"}]
    for i, raw in db.execute("select topic_id,data from messages where topic_id in (" +
                             ",".join(map(str, tf_ids)) + ") order by timestamp"):
        m = deserialize_message(raw, types[i])
        for tr in m.transforms:
            (full.set_transform_static if topics[i][0] == "/tf_static" else full.set_transform)(tr, "bag")

    def pose(buf, sec=None):
        ts = rclpy.time.Time() if sec is None else rclpy.time.Time(nanoseconds=round(sec * 1e9))
        tr = buf.lookup_transform("map", "body", ts)
        a = tr.transform
        return [a.translation.x, a.translation.y, yaw(a.rotation), stamp(tr.header.stamp)]

    def transform(xy, p):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        c, s = math.cos(p[2]), math.sin(p[2])
        return xy @ np.array([[c, s], [-s, c]]) + p[:2]

    def inverse(xy, p):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2) - p[:2]
        c, s = math.cos(p[2]), math.sin(p[2])
        return xy @ np.array([[c, -s], [s, c]])

    def nearest(series, t):
        if not series:
            return None
        j = bisect.bisect_right([v["t"] for v in series], t) - 1
        return series[max(0, j)]

    def dense(xy, spacing=.025):
        out = []
        for a, b in zip(xy, xy[1:]):
            n = max(1, math.ceil(float(np.linalg.norm(b-a)) / spacing))
            out.extend(a + (b-a)*j/n for j in range(n))
        return np.asarray(out + [xy[-1]])

    records = {n: [] for n in wanted if n not in {"/tf", "/tf_static"}}
    grids = []; paths = []; scans = []
    for ts, i, raw in db.execute("select timestamp,topic_id,data from messages order by timestamp"):
        if i not in types:
            continue
        n = topics[i][0]; m = deserialize_message(raw, types[i]); t = (ts-start)/1e9
        if n in {"/tf", "/tf_static"}:
            for tr in m.transforms:
                (online.set_transform_static if n == "/tf_static" else online.set_transform)(tr, "bag")
            continue
        row = {"t": t}
        if n in {"/control/status", "/planning/local_replan_status", "/safety/event", "/avoidance/proximity_status"}:
            row.update(json.loads(m.data))
        elif n in {"/cmd_vel", "/cmd_vel_safe", "/control/body_cmd", "/odom"}:
            tw = m.twist.twist if n == "/odom" else m.twist if n == "/control/body_cmd" else m
            row.update(v=tw.linear.x, w=tw.angular.z)
        elif n == "/control/tracking_error":
            row.update(lateral=m.vector.x, heading=m.vector.y, third=m.vector.z)
        elif n == "/mission/route_enable":
            row["enabled"] = m.data
        elif n in {"/avoidance/local_costmap", "/scan", "/planning/local_trajectory"}:
            row.update(stamp=stamp(m.header.stamp)-t0, frame=m.header.frame_id)
            try:
                row["pose_latest"] = pose(online)
                row["pose_stamp"] = pose(full, stamp(m.header.stamp))
            except Exception as exc:
                row["tf_error"] = str(exc)
            if n == "/avoidance/local_costmap":
                d = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
                rr, cc = np.nonzero(d >= 50)
                xy = np.column_stack((m.info.origin.position.x+(cc+.5)*m.info.resolution,
                                      m.info.origin.position.y+(rr+.5)*m.info.resolution))
                raw_xy = xy[d[rr, cc] >= 100]
                clip = xy[(xy[:, 0] >= .05) & (xy[:, 0] <= 4) & (np.abs(xy[:, 1]) <= 2.5)]
                sampled = occupied_grid_cell_centers(m.data, width=m.info.width, height=m.info.height,
                    resolution_m=m.info.resolution, origin_x_m=m.info.origin.position.x,
                    origin_y_m=m.info.origin.position.y, occupancy_threshold=50,
                    x_min_m=.05, x_max_m=4., y_half_width_m=2.5, max_points=2000)
                row.update(occupied_count=len(xy), clipped_count=len(clip), sampled_count=len(sampled),
                           raw_count=len(raw_xy), resolution=m.info.resolution)
                row.update(xy=xy, raw_xy=raw_xy, sampled=np.asarray(sampled).reshape(-1, 2), data=d,
                           origin=[m.info.origin.position.x, m.info.origin.position.y])
                grids.append(row)
            elif n == "/scan":
                ranges = np.asarray(m.ranges); a = m.angle_min + np.arange(len(ranges))*m.angle_increment
                keep = np.isfinite(ranges) & (ranges >= m.range_min) & (ranges <= min(m.range_max, 6))
                row["xy"] = np.column_stack((ranges[keep]*np.cos(a[keep]), ranges[keep]*np.sin(a[keep])))
                row["finite_count"] = int(keep.sum()); scans.append(row)
            else:
                row["geometry"] = np.asarray([[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in m.poses])
                paths.append(row)
        records[n].append(row)
    print(f"Read {len(grids)} grids, {len(paths)} paths and {len(scans)} scans", flush=True)

    run_rel = "log/retest_obstacle_full_loop_050straight_030turn_20261102_200354_970032218"
    params = yaml.safe_load((args.baseline_root / run_rel / "replanner.yaml").read_text())["/rebuild_local_replanner"]["ros__parameters"]
    map_path = args.baseline_root / "maps/indoor_manual_final/map.yaml"
    static = OccupancyGridMap.from_yaml(map_path)
    fields = {f.name for f in dataclasses.fields(LocalReplanConfig)}
    kw = {k:v for k,v in params.items() if k in fields}
    kw["goal_heading_tolerance_rad"] = math.radians(params["goal_heading_tolerance_deg"])
    cfg = LocalReplanConfig(**kw)
    lp = LocalTrajectoryPlanner(static, cfg)
    ref_data = yaml.safe_load((args.baseline_root / "artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml").read_text())["points"]
    reference = np.asarray([[p["x"], p["y"], p["yaw"]] for p in ref_data])

    # Compare path against full recorded grid as well as actual sampled input.
    path_metrics = []; fixtures = []
    fixture_ids = {id(min(paths, key=lambda p: abs(p['t']-t))) for t in [34.82, 66.33, 171.327, 172.806, 173.234]}
    status_rows = records["/planning/local_replan_status"]
    for p in paths:
        xy = p["geometry"]
        if len(xy) < 2 or "pose_latest" not in p:
            continue
        following = [s for s in status_rows if 0 <= s["t"]-p["t"] < .1 and not s.get("stop_requested")]
        st = following[0] if following else nearest(status_rows, p["t"])
        input_t = p["t"] - (st.get("planning_time_ms", 0)/1000) - st.get("obstacle_age_s", 0)
        # The recorder time is not the subscriber callback time; retain residual.
        planning_start_t = p['t'] - st.get('planning_time_ms', 0)/1000
        causal_grids = [z for z in grids if z['t'] <= planning_start_t]
        if not causal_grids:
            continue
        g = min(causal_grids, key=lambda z: abs(z["t"]-input_t))
        if "pose_latest" not in g:
            continue
        local_ref = tuple(PathPoint(*v) for v in xy)
        cp = PathPoint(*p["pose_latest"][:3])
        crop = _crop_planning_map(static, cp, local_ref,
            margin_m=cfg.search_padding_m+cfg.inflation_radius_m+cfg.step_length_m)
        live, _ = _overlay_obstacles(crop, tuple(map(tuple, transform(g["sampled"], g["pose_latest"]))))
        hp = lp._planner(live, local_ref)
        body = inverse(dense(xy[:, :2]), g["pose_stamp"] if "pose_stamp" in g else g["pose_latest"])
        col = np.floor((body[:, 0]-g["origin"][0])/g["resolution"]).astype(int)
        row = np.floor((body[:, 1]-g["origin"][1])/g["resolution"]).astype(int)
        inside = (col>=0)&(row>=0)&(col<g["data"].shape[1])&(row<g["data"].shape[0])
        values = g["data"][row[inside], col[inside]]
        ds = np.linalg.norm(np.diff(xy[:, :2], axis=0), axis=1)
        k = np.asarray([wrap(v) for v in np.diff(xy[:, 2])]) / np.maximum(ds, 1e-6)
        near = xy[np.argmin(np.linalg.norm(xy[:, :2]-p["pose_latest"][:2], axis=1))]
        pm = {"t": p["t"], "status": st.get("status"), "grid_t": g["t"],
              "input_selection_residual_s": g["t"]-input_t,
              "sampled_baseline_path_clear": hp.path_is_navigable(local_ref),
              "full_grid_path_points": len(values), "full_grid_blocked_samples": int((values>=50).sum()),
              "path_length_m": float(ds.sum()), "max_abs_curvature": float(np.max(np.abs(k))),
              "pose_to_path_m": float(np.linalg.norm(near[:2]-p["pose_latest"][:2])),
              "pose_to_path_heading_deg": math.degrees(wrap(near[2]-p["pose_latest"][2]))}
        path_metrics.append(pm)
        if id(p) in fixture_ids:
            fixtures.append({'t':p['t'],'status':st,'input_selection_residual_s':g['t']-input_t,
                'pose':xy[0].tolist(),'dynamic_points':transform(g['sampled'],g['pose_latest']).tolist(),
                'full_points':transform(g['xy'],g['pose_latest']).tolist(),
                'full_grid_pose':g['pose_latest'][:3],
                'full_grid_bounds':[g['origin'][0],g['origin'][1],
                    g['origin'][0]+g['data'].shape[1]*g['resolution'],
                    g['origin'][1]+g['data'].shape[0]*g['resolution']],
                'published_path':xy.tolist(),'reference_index':st.get('reference_start_index',0)})

    grid_metrics = []; snapshots = []
    selected = [36, 40, 45, 68, 71, 73, 75, 76.4, 172, 175, 177, 178, 185.4, 199, 213.4]
    selected_ids = {id(min(grids, key=lambda x: abs(x["t"]-t))) for t in selected}
    for g in grids:
        if "pose_latest" not in g or "pose_stamp" not in g:
            continue
        p = g["pose_latest"]; ps = g["pose_stamp"]
        body = inverse(transform(g["raw_xy"], ps), p)
        delta = np.maximum(np.abs(body)-[.36, .25], 0)
        rect_gap = float(np.linalg.norm(delta, axis=1).min() - g["resolution"]*math.sqrt(2)/2) if len(body) else None
        shift = np.linalg.norm(transform(g["sampled"], p)-transform(g["sampled"], ps), axis=1)
        row = {"t":g["t"], "header_age_s":g["t"]-g["stamp"],
               "latest_tf_age_s": t0+g["t"]-p[3],
               "tf_translation_difference_m":math.hypot(p[0]-ps[0], p[1]-ps[1]),
               "tf_heading_difference_deg":math.degrees(wrap(p[2]-ps[2])),
               "obstacle_shift_p95_m":float(np.percentile(shift,95)) if len(shift) else 0,
               "clipped_count":g["clipped_count"],"sampled_count":g["sampled_count"],
               "rectangle_to_raw_grid_gap_m":rect_gap,
               "pose":p[:3]}
        if id(g) in selected_ids:
            idx = int(np.argmin(np.linalg.norm(reference[:,:2]-p[:2],axis=1)))
            end = idx+1; length=0
            while end < len(reference)-1 and length<3:
                length += float(np.linalg.norm(reference[end,:2]-reference[end-1,:2]));end+=1
            local_ref=tuple(PathPoint(*v) for v in reference[idx:end+1]);cp=PathPoint(*p[:3])
            crop=_crop_planning_map(static,cp,local_ref,margin_m=1.74)
            live,_=_overlay_obstacles(crop,tuple(map(tuple,transform(g['sampled'],p))))
            hp=lp._planner(live,local_ref);seed=_Node(*p[:3],len(hp._curvatures)//2)
            row['start_clear']=hp.path_is_navigable((cp,))
            row['initial_clear']=[hp._motion_clearance_cost(seed,j) is not None for j in hp._successor_curvatures(seed.curvature_index)]
            path=nearest(paths,g['t']);scan=nearest(scans,g['t'])
            snapshots.append({'t':g['t'],'pose':p[:3],'raw_obstacles':transform(g['raw_xy'],ps).tolist(),
                'inflated_obstacles':transform(g['xy'],ps).tolist(),
                'path_t':path['t'] if path else None,
                'path':path['geometry'].tolist() if path is not None else [],
                'reference':reference[max(0,idx-25):end+25].tolist(),
                'scan':transform(scan['xy'],scan['pose_stamp']).tolist() if scan and 'pose_stamp' in scan else [],
                'diagnostics':row})
        grid_metrics.append(row)

    def plain(v):
        if isinstance(v,np.ndarray):return v.tolist()
        if isinstance(v,np.generic):return v.item()
        raise TypeError(type(v).__name__)

    # Do not duplicate grid/scan arrays in the time-series artifact.
    series={n:rs for n,rs in records.items() if n not in {'/scan','/avoidance/local_costmap','/planning/local_trajectory'}}
    result={'bag':str(args.bag),'baseline_root':str(args.baseline_root),'t0_epoch_s':t0,
            'limitations':['Recorder arrival order approximates subscriber order; input selection residual is retained.',
                           'No original 3D cloud or wheel steering feedback in this recording.',
                           'Raw obstacle cells are occupied scan cells, not surveyed physical obstacle geometry.',
                           'Path validation and snapshots are diagnostics, not a closed-loop execution proof.'],
            'grid_metrics':grid_metrics,'path_metrics':path_metrics,'series':series}
    (args.output/'origin_analysis.json').write_text(json.dumps(result,default=plain),encoding='utf-8')
    (args.output/'origin_snapshots.json').write_text(json.dumps(snapshots,default=plain),encoding='utf-8')
    (args.output/'origin_fixtures.json').write_text(json.dumps(fixtures,default=plain),encoding='utf-8')
    (args.output/'path_geometry.json').write_text(json.dumps([
        {'t':p['t'],'geometry':p['geometry']} for p in paths],default=plain),encoding='utf-8')
    summary={'grid_count':len(grid_metrics),'path_count':len(path_metrics),
             'grids_over_2000':sum(g['clipped_count']>2000 for g in grid_metrics),
             'path_sampled_collisions':sum(not p['sampled_baseline_path_clear'] for p in path_metrics),
             'path_full_grid_collisions':sum(p['full_grid_blocked_samples']>0 for p in path_metrics)}
    for key in ['header_age_s','tf_translation_difference_m','obstacle_shift_p95_m','rectangle_to_raw_grid_gap_m']:
        v=[g[key] for g in grid_metrics if g.get(key) is not None]
        summary[key]={str(q):float(np.percentile(v,q)) for q in [0,50,95,100]}
    (args.output/'origin_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary),flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
