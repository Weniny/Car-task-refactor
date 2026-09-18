#!/usr/bin/env python3
"""Summarize a recorded navigation run without publishing ROS messages."""

from __future__ import annotations

import argparse
import json
import sqlite3
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean, median

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/control/status",
    "/control/tracking_error",
    "/control/body_cmd",
    "/planning/local_replan_status",
    "/planning/local_stop_request",
    "/avoidance/stop_request",
    "/safety/event",
    "/cmd_vel_safe",
    "/cmd_vel",
    "/odom",
}


def json_data(message):
    try:
        return json.loads(message.data)
    except (ValueError, TypeError):
        return {"status": "INVALID_JSON"}


def percentile(values, percent):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percent / 100)
    return ordered[index]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or .db3 file")
    args = parser.parse_args()
    databases = sorted(args.bag.glob("*.db3")) if args.bag.is_dir() else [args.bag]
    if len(databases) != 1 or not databases[0].is_file():
        parser.error("expected one .db3 file")

    db = sqlite3.connect(f"file:{databases[0].resolve()}?mode=ro", uri=True)
    try:
        topic_info = {
            row[0]: (row[1], get_message(row[2]))
            for row in db.execute("SELECT id, name, type FROM topics")
            if row[1] in TOPICS
        }
        if "/control/status" not in {name for name, _ in topic_info.values()}:
            parser.error("bag lacks /control/status")

        state = {
            "local_stop": False,
            "avoidance_stop": False,
            "replan": "UNKNOWN",
            "safety": "UNKNOWN",
            "safety_reasons": (),
            "safe_v": 0.0,
            "safe_w": 0.0,
            "raw_v": 0.0,
            "raw_w": 0.0,
            "lateral_error": 0.0,
            "heading_error_deg": 0.0,
            "chassis_v": 0.0,
            "odom_v": 0.0,
            "odom_w": 0.0,
        }
        samples = []
        planning_ms = []
        replanner_details = Counter()
        start_ns = None
        end_ns = None
        for timestamp, topic_id, raw in db.execute(
            "SELECT timestamp, topic_id, data FROM messages ORDER BY timestamp"
        ):
            info = topic_info.get(topic_id)
            if info is None:
                continue
            name, message_class = info
            message = deserialize_message(raw, message_class)
            if start_ns is None:
                start_ns = timestamp
            end_ns = timestamp
            t = (timestamp - start_ns) / 1e9

            if name == "/planning/local_stop_request":
                state["local_stop"] = bool(message.data)
            elif name == "/avoidance/stop_request":
                state["avoidance_stop"] = bool(message.data)
            elif name == "/planning/local_replan_status":
                value = json_data(message)
                state["replan"] = value.get("status", "UNKNOWN")
                if isinstance(value.get("planning_time_ms"), (int, float)):
                    planning_ms.append(float(value["planning_time_ms"]))
                if value.get("detail"):
                    replanner_details[(state["replan"], value["detail"])] += 1
            elif name == "/safety/event":
                value = json_data(message)
                state["safety"] = value.get("status", "UNKNOWN")
                state["safety_reasons"] = tuple(value.get("reasons") or ())
            elif name == "/cmd_vel_safe":
                state["safe_v"] = float(message.linear.x)
                state["safe_w"] = float(message.angular.z)
            elif name == "/control/body_cmd":
                state["raw_v"] = float(message.twist.linear.x)
                state["raw_w"] = float(message.twist.angular.z)
            elif name == "/control/tracking_error":
                state["lateral_error"] = float(message.vector.x)
                state["heading_error_deg"] = float(message.vector.y) * 180.0 / 3.141592653589793
            elif name == "/cmd_vel":
                state["chassis_v"] = float(message.linear.x)
            elif name == "/odom":
                state["odom_v"] = float(message.twist.twist.linear.x)
                state["odom_w"] = float(message.twist.twist.angular.z)
            elif name == "/control/status":
                value = json_data(message)
                samples.append(
                    {
                        "t": t,
                        "status": value.get("status", "UNKNOWN"),
                        "enabled": value.get("route_enabled") is True,
                        "progress": value.get("route_progress_index", -1),
                        **state,
                    }
                )
    finally:
        db.close()

    enabled = [sample for sample in samples if sample["enabled"]]
    if not enabled:
        print("No route-enabled samples found")
        return

    begin, finish = enabled[0]["t"], enabled[-1]["t"]
    print(f"bag duration: {(end_ns - start_ns) / 1e9:.1f}s")
    print(
        f"route enabled: {begin:.1f}..{finish:.1f}s ({finish - begin:.1f}s), "
        f"progress {enabled[0]['progress']}..{enabled[-1]['progress']}"
    )
    for label, key in (("controller", "status"), ("replanner", "replan"), ("safety", "safety")):
        counts = Counter(sample[key] for sample in enabled)
        print(f"{label}: {counts.most_common()}")
    reasons = Counter(
        reason for sample in enabled for reason in sample["safety_reasons"]
    )
    print(f"safety reasons: {reasons.most_common()}")
    print(
        "stop samples: "
        f"local={sum(s['local_stop'] for s in enabled)}/{len(enabled)}, "
        f"avoidance={sum(s['avoidance_stop'] for s in enabled)}/{len(enabled)}"
    )
    print(
        f"planning time: median={median(planning_ms):.1f}ms "
        f"p95={percentile(planning_ms, 95):.1f}ms "
        f"max={max(planning_ms):.1f}ms"
        if planning_ms else "planning time: unavailable"
    )
    print(f"replanner failure details: {replanner_details.most_common(5)}")

    bins = defaultdict(list)
    for sample in enabled:
        bins[int((sample["t"] - begin) // 10)].append(sample)
    print("\n10s bins (time, progress, safe/odom speed, local/avoidance stop %, replan/safety):")
    for index, group in sorted(bins.items()):
        replan = Counter(sample["replan"] for sample in group).most_common(1)[0][0]
        safety = Counter(sample["safety"] for sample in group).most_common(1)[0][0]
        print(
            f"{begin + 10 * index:6.1f}..{begin + 10 * (index + 1):6.1f} "
            f"{group[0]['progress']:4}..{group[-1]['progress']:4} "
            f"safe={mean(s['safe_v'] for s in group):.3f} "
            f"odom={mean(s['odom_v'] for s in group):.3f} "
            f"raw={mean(s['raw_v'] for s in group):.3f} "
            f"err={max(abs(s['lateral_error']) for s in group):.2f}m/"
            f"{max(abs(s['heading_error_deg']) for s in group):.1f}deg "
            f"stop={100 * mean(s['local_stop'] for s in group):.0f}/"
            f"{100 * mean(s['avoidance_stop'] for s in group):.0f}% "
            f"{replan}/{safety}"
        )

    print("\nstationary episodes (>5s while route enabled):")
    episodes = []
    current = []
    for sample in enabled:
        stalled = abs(sample["odom_v"]) < 0.02
        if stalled and (not current or sample["t"] - current[-1]["t"] < 0.3):
            current.append(sample)
        else:
            if current:
                episodes.append(current)
            current = [sample] if stalled else []
    if current:
        episodes.append(current)
    for episode in episodes:
        if episode[-1]["t"] - episode[0]["t"] < 5:
            continue
        print(
            f"{episode[0]['t']:.1f}..{episode[-1]['t']:.1f}s "
            f"({episode[-1]['t'] - episode[0]['t']:.1f}s) "
            f"progress={episode[0]['progress']}..{episode[-1]['progress']} "
            f"safe_v={mean(s['safe_v'] for s in episode):.3f} "
            f"raw_v={mean(s['raw_v'] for s in episode):.3f} "
            f"safe_abs_w={mean(abs(s['safe_w']) for s in episode):.3f} "
            f"avoidance={100 * mean(s['avoidance_stop'] for s in episode):.0f}% "
            f"max_error={max(abs(s['lateral_error']) for s in episode):.2f}m/"
            f"{max(abs(s['heading_error_deg']) for s in episode):.1f}deg "
            f"replan={Counter(s['replan'] for s in episode).most_common(2)} "
            f"safety={Counter(s['safety'] for s in episode).most_common(2)} "
            f"reasons={Counter(r for s in episode for r in s['safety_reasons']).most_common(3)}"
        )

    print("\ntracking recovery episodes (>2s):")
    episodes = []
    current = []
    for sample in enabled:
        recovering = sample["safety"] == "SAFE_RECOVERY"
        if recovering and (not current or sample["t"] - current[-1]["t"] < 0.3):
            current.append(sample)
        else:
            if current:
                episodes.append(current)
            current = [sample] if recovering else []
    if current:
        episodes.append(current)
    for episode in episodes:
        if episode[-1]["t"] - episode[0]["t"] < 2:
            continue
        print(
            f"{episode[0]['t']:.1f}..{episode[-1]['t']:.1f}s "
            f"progress={episode[0]['progress']}..{episode[-1]['progress']} "
            f"raw_v={mean(s['raw_v'] for s in episode):.3f} "
            f"safe_v={mean(s['safe_v'] for s in episode):.3f} "
            f"max_error={max(abs(s['lateral_error']) for s in episode):.2f}m/"
            f"{max(abs(s['heading_error_deg']) for s in episode):.1f}deg"
        )


if __name__ == "__main__":
    main()
