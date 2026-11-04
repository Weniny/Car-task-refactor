"""Small, checked start capture for a car parked behind a finite reference.

The start envelope is the existing preflight envelope, not a wider arrival
tolerance. This helper never moves localization or changes the global route.
"""
import math
from competition_planning.semantic_planner import PathPoint

START_POSITION_TOLERANCE_M = 0.30
START_HEADING_TOLERANCE_DEG = 12.0


def start_offsets(origin, pose):
    c,s=math.cos(origin.yaw),math.sin(origin.yaw)
    dx,dy=pose.x-origin.x,pose.y-origin.y
    return (c*dx+s*dy,-s*dx+c*dy,
        math.atan2(math.sin(pose.yaw-origin.yaw),math.cos(pose.yaw-origin.yaw)))


def startup_capture_path(reference, pose, *, goal_index, sample_spacing_m,
                         min_turning_radius_m, capture_half_width_m):
    """Return a tangent-continuous cubic, or no candidate outside the start envelope.

Collision checking is the caller's responsibility, using the current inflated
map. Only the initial behind-start case is handled here.
"""
    if not 0 < goal_index < len(reference):
        return ()
    along,lateral,heading=start_offsets(reference[0],pose)
    if (not all(math.isfinite(v) for v in (along,lateral,heading))
        or along >= -1e-6
        or math.hypot(along,lateral) > START_POSITION_TOLERANCE_M+1e-9
        or abs(lateral) > capture_half_width_m
        or abs(heading) > math.radians(START_HEADING_TOLERANCE_DEG)+1e-9):
        return ()
    if sample_spacing_m <= 0 or min_turning_radius_m <= 0:
        raise ValueError("invalid capture geometry bounds")
    goal=reference[goal_index]
    chord=math.hypot(goal.x-pose.x,goal.y-pose.y)
    if chord < 2*min_turning_radius_m:
        return ()
    h=chord/3.0
    p0=(pose.x,pose.y)
    p1=(pose.x+h*math.cos(pose.yaw),pose.y+h*math.sin(pose.yaw))
    p2=(goal.x-h*math.cos(goal.yaw),goal.y-h*math.sin(goal.yaw))
    p3=(goal.x,goal.y)
    # Dense analytic curvature checks and a <= sample-spacing control polygon
    # bound avoid hiding a sharp bend between output points.
    polygon=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip((p0,p1,p2),(p1,p2,p3)))
    steps=max(2,math.ceil(polygon/sample_spacing_m))
    path=[]
    for j in range(2*steps+1):
        u=j/(2*steps);v=1-u
        x=v**3*p0[0]+3*v*v*u*p1[0]+3*v*u*u*p2[0]+u**3*p3[0]
        y=v**3*p0[1]+3*v*v*u*p1[1]+3*v*u*u*p2[1]+u**3*p3[1]
        dx=3*(v*v*(p1[0]-p0[0])+2*v*u*(p2[0]-p1[0])+u*u*(p3[0]-p2[0]))
        dy=3*(v*v*(p1[1]-p0[1])+2*v*u*(p2[1]-p1[1])+u*u*(p3[1]-p2[1]))
        ddx=6*(v*(p2[0]-2*p1[0]+p0[0])+u*(p3[0]-2*p2[0]+p1[0]))
        ddy=6*(v*(p2[1]-2*p1[1]+p0[1])+u*(p3[1]-2*p2[1]+p1[1]))
        norm=math.hypot(dx,dy)
        if norm < 1e-9 or abs(dx*ddy-dy*ddx)/norm**3 > 1/min_turning_radius_m+1e-9:
            return ()
        if j%2==0:path.append(PathPoint(x,y,math.atan2(dy,dx)))
    path[0]=PathPoint(pose.x,pose.y,pose.yaw)
    path[-1]=goal
    for a,b in zip(path,path[1:]):
        ds=math.hypot(b.x-a.x,b.y-a.y)
        dtheta=math.atan2(math.sin(b.yaw-a.yaw),math.cos(b.yaw-a.yaw))
        if ds < 1e-9 or abs(dtheta)/ds > 1/min_turning_radius_m+1e-9:
            return ()
    return tuple(path)
