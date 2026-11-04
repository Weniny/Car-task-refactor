#!/usr/bin/env python3
"""Fault-inject bounded search failures into the actual isolated ROS adapter.

Only the search is replaced; map checks, fallback, guards and typed publishers
are real. No controller/chassis process or route-enable publisher is created.
"""
import json
import math
import os
from pathlib import Path
import tempfile
import time

if os.environ.get('ROS_DOMAIN_ID')!='94' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires domain 94 and localhost only')
root=Path(os.environ['COMPETITION_FORWARD_WS'])
import yaml
import rclpy
from geometry_msgs.msg import TransformStamped
from competition_planning.local_replanner_node import LocalReplannerNode
from competition_planning.local_trajectory_planner import LocalTrajectoryPlanner
from competition_planning.forward_rejoin import ObservationWindow
from competition_planning.hybrid_astar_planner import HybridAStarTimeout
from competition_planning.semantic_planner import PathPoint
from competition_planning.path_guard import PathGuard

old=Path('/home/agilex/competition_forward_rejoin_20261103_ws')
run=old/'log/retest_obstacle_forward_rejoin_inflation025_retest_full_loop_20261104_122749_172705551'
params=next(iter(yaml.safe_load((run/'replanner.yaml').read_text()).values()))['ros__parameters']
params={k:v.replace(str(old),str(root)) if isinstance(v,str) else v for k,v in params.items()}
params.update(behavior_guard_enabled=True,guard_max_speed_mps=.5,guard_acceleration_mps2=.12,
    guard_deceleration_mps2=.18,guard_command_delay_s=.15)
snapshots=json.loads((root/'checks/recorded_behavior_snapshots.json').read_text())
sample=lambda t:min(snapshots,key=lambda r:abs(r['t']-t))

class Spy:
    def __init__(self,publisher):self.publisher=publisher;self.messages=[]
    def publish(self,message):
        self.publisher.publish(message);self.messages.append(message)

with tempfile.TemporaryDirectory(prefix='behavior_check_') as tmp:
    config=Path(tmp)/'params.yaml';config.write_text(yaml.safe_dump({'/local_replanner':{'ros__parameters':params}}))
    rclpy.init(args=['--ros-args','--params-file',str(config)])
    node=LocalReplannerNode()
    try:
        for name in ['_path_publisher','_guard_publisher','_stop_publisher','_status_publisher']:
            setattr(node,name,Spy(getattr(node,name)))
        results=[]
        for name,scene,previous in [('far',sample(79.05),sample(79.05)),('near',sample(113.4),sample(112.9))]:
            now=node.get_clock().now();sec=now.nanoseconds/1e9
            tr=TransformStamped();tr.header.stamp=now.to_msg();tr.header.frame_id='map';tr.child_frame_id='body'
            tr.transform.translation.x=scene['pose'][0];tr.transform.translation.y=scene['pose'][1]
            tr.transform.rotation.z=math.sin(scene['pose'][2]/2);tr.transform.rotation.w=math.cos(scene['pose'][2]/2)
            node._tf_buffer.set_transform(tr,'isolated-failure-check')
            node._planner=LocalTrajectoryPlanner(node._static_map,node._config)
            node._planner._last_safe_path=tuple(PathPoint(*p) for p in previous['path'])
            node._planner._last_safe_rejoin_index=previous['rejoin_index']
            node._planner._forward_rejoin_target_index=previous['rejoin_index']
            def fail(**kwargs):
                time.sleep(.4);raise HybridAStarTimeout('injected bounded search timeout')
            node._planner.plan=fail
            node._previous_reference_index=scene['reference_start_index']
            node._observation_window=ObservationWindow(*scene['pose'],-.5,-2.5,5.5,2.5)
            node._obstacle_points_map=tuple(map(tuple,scene['inflated_world']))
            node._obstacle_frame='body';node._obstacle_received_s=sec;node._obstacle_header_stamp_s=sec
            node._odom_received_s=sec;node._odom_header_stamp_s=sec;node._linear_speed_mps=.5
            node._controller_status={'status':'TRACKING','route_enabled':True};node._controller_status_received_s=sec
            node._planning_cycle()
            status=json.loads(node._status_publisher.messages[-1].data)
            path=node._path_publisher.messages[-1];stop=node._stop_publisher.messages[-1].data
            if name=='far':
                assert status['status']=='VALIDATED_PREFIX' and stop is False and len(path.poses)>1,status
                ticket=PathGuard.parse(node._guard_publisher.messages[-1].data)
                assert ticket.path_stamp_ns==path.header.stamp.sec*10**9+path.header.stamp.nanosec
                assert ticket.continuation and ticket.speed_limit_mps>0
            else:
                assert stop is True and not path.poses,status
            results.append({'case':name,'status':status['status'],'stop_requested':stop,
                'path_points':len(path.poses),'clear_distance_m':status.get('continuation_clear_distance_m')})
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        result={'status':'PASS','domain':94,'localhost_only':True,'cmd_vel_publishers':0,
            'fault':'search only forced to fail after 0.4s; real node fallback and publishers', 'cases':results}
        (root/'log/behavior_ros_failures_20261104.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        node.destroy_node();rclpy.shutdown()
