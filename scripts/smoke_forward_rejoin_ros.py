#!/usr/bin/env python3
"""Isolated ROS interface check with synthetic zero-motion inputs; no chassis."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='94':raise SystemExit('Requires isolated ROS_DOMAIN_ID=94')
root=Path(os.environ['COMPETITION_FORWARD_WS'])
launch_file=os.environ.get('SMOKE_LAUNCH_FILE','forward_rejoin_050_030.launch.py')
expected_inflation=float(os.environ.get('SMOKE_EXPECTED_INFLATION','0.44'))
out=root/('log/interface_smoke_20261104_inflation025' if expected_inflation==.25 else 'log/interface_smoke_20261104');out.mkdir(parents=True,exist_ok=True)
log=(out/'launch.log').open('w')
proc=subprocess.Popen(['ros2','launch','forward_rejoin_bringup',launch_file,'start_chassis_adapter:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
node=None
try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import TransformStamped,Twist
    from nav_msgs.msg import OccupancyGrid,Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from ranger_msgs.msg import SystemState,MotionState
    from tf2_ros import TransformBroadcaster
    rclpy.init();node=Node('forward_rejoin_interface_smoke')
    observed={'control':None,'planner':None,'safety':None,'safe_messages':0,'nonzero_safe':0}
    def status(key):
        return lambda m:observed.__setitem__(key,json.loads(m.data))
    node.create_subscription(String,'/control/status',status('control'),10)
    node.create_subscription(String,'/planning/local_replan_status',status('planner'),10)
    node.create_subscription(String,'/safety/event',status('safety'),10)
    def safe(m):
        observed['safe_messages']+=1
        if max(abs(m.linear.x),abs(m.linear.y),abs(m.angular.z))>1e-6:observed['nonzero_safe']+=1
    node.create_subscription(Twist,'/cmd_vel_safe',safe,10)
    def grid_radius(message):
        raw=[i for i,v in enumerate(message.data) if v==100]
        occupied=[i for i,v in enumerate(message.data) if v>=50]
        if len(raw)==1:
            x,y=raw[0]%message.info.width,raw[0]//message.info.width
            observed['grid_inflation_radius_m']=max(math.hypot(i%message.info.width-x,i//message.info.width-y) for i in occupied)*message.info.resolution
    node.create_subscription(OccupancyGrid,'/avoidance/local_costmap',grid_radius,10)
    tf=TransformBroadcaster(node)
    odom_pub=node.create_publisher(Odometry,'/odom',10)
    grid_pub=node.create_publisher(OccupancyGrid,'/avoidance/local_costmap',10)
    scan_pub=node.create_publisher(LaserScan,'/scan',10)
    sys_pub=node.create_publisher(SystemState,'/system_state',10)
    mode_pub=node.create_publisher(MotionState,'/motion_state',10)
    started=time.monotonic()
    while time.monotonic()-started<40:
        if proc.poll() is not None:raise RuntimeError('Launch exited: '+str(proc.returncode))
        stamp=node.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.stamp=stamp;transform.header.frame_id='map';transform.child_frame_id='body'
        transform.transform.translation.x=-16.083;transform.transform.translation.y=2.309
        transform.transform.rotation.z=math.sin(3.016522/2);transform.transform.rotation.w=math.cos(3.016522/2)
        tf.sendTransform(transform)
        odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='body';odom_pub.publish(odom)
        grid=OccupancyGrid();grid.header.stamp=stamp;grid.header.frame_id='body';grid.info.resolution=.05;grid.info.width=120;grid.info.height=100
        grid.info.origin.position.x=-.5;grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;grid.data=[0]*12000;grid_pub.publish(grid)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='body';scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/360
        scan.range_min=.1;scan.range_max=6.;scan.ranges=[float('inf')]*361
        if expected_inflation==.25:
            ranges=list(scan.ranges);ranges[254]=2.5;scan.ranges=ranges
        scan_pub.publish(scan)
        system=SystemState();system.header.stamp=stamp;system.control_mode=1;system.error_code=0;system.motion_mode=0;system.battery_voltage=49.5;sys_pub.publish(system)
        mode=MotionState();mode.header.stamp=stamp;mode.motion_mode=0;mode_pub.publish(mode)
        rclpy.spin_once(node,timeout_sec=.05)
        c,p=observed['control'],observed['planner']
        if (time.monotonic()-started>8 and c and p and c.get('status')=='ROUTE_DISABLED'
            and c.get('route_enabled') is False and p.get('status')=='REFERENCE_CLEAR' and observed['safe_messages']>=10
            and (expected_inflation!=.25 or 'grid_inflation_radius_m' in observed)):break
    expected={'rebuild_replan_proximity_scan','rebuild_replan_proximity_stop','rebuild_local_replanner','rebuild_replan_mppi','rebuild_replan_safety'}
    names=set(node.get_node_names());missing=expected-names
    if missing:raise RuntimeError('Missing nodes: '+str(missing))
    if observed['control'] is None or observed['control'].get('status')!='ROUTE_DISABLED':raise RuntimeError('Controller is not disarmed: '+str(observed['control']))
    if observed['planner'] is None or observed['planner'].get('status')!='REFERENCE_CLEAR':raise RuntimeError('Planner interface failed: '+str(observed['planner']))
    if observed['nonzero_safe'] or not observed['safe_messages']:raise RuntimeError('Zero-output check failed')
    if node.get_publishers_info_by_topic('/cmd_vel'):raise RuntimeError('Unexpected final chassis publisher')
    client=node.create_client(GetParameters,'/rebuild_local_replanner/get_parameters')
    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Parameter service missing')
    request=GetParameters.Request();request.names=['forward_rejoin_enabled','tracking_clearance_m','min_turning_radius_m']
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    values=future.result().values
    assert values[0].bool_value and abs(values[1].double_value-.10)<1e-9 and abs(values[2].double_value-.60)<1e-9
    client=node.create_client(GetParameters,'/rebuild_replan_proximity_stop/get_parameters')
    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Proximity parameter service missing')
    request=GetParameters.Request();request.names=['grid_inflation_radius_m']
    future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    assert abs(future.result().values[0].double_value-expected_inflation)<1e-9
    if expected_inflation==.25:assert abs(observed['grid_inflation_radius_m']-.25)<1e-6
    result={'status':'PASS','launch':launch_file,'ros_domain_id':94,'hardware_connected':False,'route_enabled':False,'cmd_vel_publishers':0,'parameters':{'forward_rejoin_enabled':True,'tracking_clearance_m':.10,'min_turning_radius_m':.60,'grid_inflation_radius_m':expected_inflation},'observations':observed}
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
    if node is not None:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    log.close()
