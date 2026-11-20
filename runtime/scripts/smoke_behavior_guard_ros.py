#!/usr/bin/env python3
"""Exercise real ROS nodes in domain 94 on localhost, with no chassis adapter."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='94' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1')
root=Path(os.environ['COMPETITION_FORWARD_WS'])
out=root/'log/behavior_guard_ros_check_20261104';out.mkdir(parents=True,exist_ok=True)
log=(out/'launch.log').open('w')
proc=subprocess.Popen(['ros2','launch','forward_rejoin_bringup',
    'forward_rejoin_behavior_050_030_inflation025.launch.py','start_chassis_adapter:=false'],
    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
node=None;paused_pid=None
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped,Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool,String
    from ranger_msgs.msg import SystemState,MotionState
    from tf2_ros import TransformBroadcaster
    import yaml
    ref=yaml.safe_load((root/'artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml').read_text())['points'][0]
    rclpy.init();node=Node('behavior_guard_ros_check')
    observed={'control':None,'planner':None,'safe':None,'nonzero_safe':0,'control_statuses':[]}
    def status(key):
        def accept(m):
            row=json.loads(m.data);observed[key]=row
            if key=='control':observed['control_statuses'].append(row['status'])
        return accept
    node.create_subscription(String,'/control/status',status('control'),10)
    node.create_subscription(String,'/planning/local_replan_status',status('planner'),10)
    def safe(m):
        observed['safe']=[m.linear.x,m.linear.y,m.angular.z]
        if max(map(abs,observed['safe']))>1e-6:observed['nonzero_safe']+=1
    node.create_subscription(Twist,'/cmd_vel_safe',safe,10)
    tf=TransformBroadcaster(node)
    odom_pub=node.create_publisher(Odometry,'/odom',10)
    scan_pub=node.create_publisher(LaserScan,'/scan',10)
    sys_pub=node.create_publisher(SystemState,'/system_state',10)
    mode_pub=node.create_publisher(MotionState,'/motion_state',10)
    route_pub=node.create_publisher(Bool,'/mission/route_enable',10)
    def tick():
        if proc.poll() is not None:raise RuntimeError('isolated launch exited')
        stamp=node.get_clock().now().to_msg()
        tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id='map';tr.child_frame_id='body'
        tr.transform.translation.x=ref['x'];tr.transform.translation.y=ref['y']
        tr.transform.rotation.z=math.sin(ref['yaw']/2);tr.transform.rotation.w=math.cos(ref['yaw']/2);tf.sendTransform(tr)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='body';odom_pub.publish(odom)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='body'
        scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/360
        scan.range_min=.1;scan.range_max=6.;scan.ranges=[float('inf')]*361;scan_pub.publish(scan)
        system=SystemState();system.header.stamp=stamp;system.control_mode=1;system.error_code=0
        system.motion_mode=0;system.battery_voltage=49.5;sys_pub.publish(system)
        mode=MotionState();mode.header.stamp=stamp;mode.motion_mode=0;mode_pub.publish(mode)
        rclpy.spin_once(node,timeout_sec=.02)
    def wait_for(test,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            tick()
            if test():return
        raise RuntimeError(label+': '+json.dumps(observed))
    wait_for(lambda:observed['control'] and observed['planner']
        and observed['control'].get('status')=='ROUTE_DISABLED'
        and observed['control'].get('local_guard_required') is True
        and observed['planner'].get('status')=='REFERENCE_CLEAR',35,'startup')
    if node.get_publishers_info_by_topic('/cmd_vel'):raise RuntimeError('unexpected chassis publisher')
    if observed['nonzero_safe']:raise RuntimeError('nonzero command before synthetic enable')
    route_pub.publish(Bool(data=True))
    wait_for(lambda:observed['control'].get('status')=='TRACKING'
        and observed['control'].get('local_guard_speed_limit_mps')==.5
        and observed['nonzero_safe']>5,12,'guarded synthetic tracking')
    # Pause only a process in the session created by this check. Never use a
    # name-based kill/stop that could affect real navigation in another domain.
    matches=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            pid=int(entry.name)
            if os.getsid(pid)!=proc.pid:continue
            cmd=(entry/'cmdline').read_bytes().replace(b'\0',b' ')
            if b'/lib/competition_planning/local_replanner_node ' in cmd:matches.append(pid)
        except (ProcessLookupError,PermissionError,FileNotFoundError):pass
    if len(matches)!=1:raise RuntimeError('cannot identify unique isolated replanner: '+str(matches))
    paused_pid=matches[0];os.kill(paused_pid,signal.SIGSTOP)
    hold_started=time.monotonic()
    wait_for(lambda:observed['control'].get('status')=='LOCAL_PLAN_GUARD_HOLD'
        and observed['safe'] is not None and max(map(abs,observed['safe']))<1e-6,2.,'expired guard hold')
    expiry_hold_delay=time.monotonic()-hold_started
    os.kill(paused_pid,signal.SIGCONT);paused_pid=None
    wait_for(lambda:observed['control'].get('status')=='TRACKING',8,'fresh full plan recovery')
    route_pub.publish(Bool(data=False))
    wait_for(lambda:observed['control'].get('status')=='ROUTE_DISABLED'
        and observed['safe'] is not None and max(map(abs,observed['safe']))<1e-6,3,'final disarm')
    assert not node.get_publishers_info_by_topic('/cmd_vel')
    result={'status':'PASS','ros_domain_id':94,'localhost_only':True,'chassis_adapter':False,
        'cmd_vel_publishers':0,'synthetic_tracking_checked':True,'guard_expiry_hold_checked':True,
        'expiry_hold_delay_s':expiry_hold_delay,'full_plan_recovery_checked':True,
        'final_route_enabled':False,'observations':observed}
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
    if paused_pid is not None:
        try:os.kill(paused_pid,signal.SIGCONT)
        except ProcessLookupError:pass
    if node is not None:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    log.close()
