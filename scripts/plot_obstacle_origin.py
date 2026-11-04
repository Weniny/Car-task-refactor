#!/usr/bin/env python3
"""Render static, shareable figures from offline origin-analysis JSON files."""
import argparse
import json
import math
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',required=True,type=Path)
    args=ap.parse_args()
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    r=args.input
    a=json.loads((r/'origin_analysis.json').read_text())
    snaps=json.loads((r/'origin_snapshots.json').read_text())
    paths=json.loads((r/'path_geometry.json').read_text())
    def inv(x,p):
        x=np.asarray(x).reshape(-1,2)-p[:2];c,s=math.cos(p[2]),math.sin(p[2])
        return x@np.array([[c,-s],[s,c]])
    fig,axs=plt.subplots(1,3,figsize=(16,6),constrained_layout=True)
    for ax,(lo,hi,stop,title) in zip(axs,[(34,46,45,'First transient stop'),(65,78,76.4,'First long stop'),(170,180,178,'Second long stop')]):
        snap=min(snaps,key=lambda x:abs(x['t']-stop));p=snap['pose']
        xy=inv(snap['inflated_obstacles'],p);ax.scatter(xy[:,0],xy[:,1],s=4,c='#d8d8df',label='Inflated cells')
        xy=inv(snap['raw_obstacles'],p);ax.scatter(xy[:,0],xy[:,1],s=6,c='#33343b',label='Raw occupied cells')
        ref=np.asarray(snap['reference']);xy=inv(ref[:,:2],p);ax.plot(xy[:,0],xy[:,1],c='#3274ad',lw=2,label='Reference')
        track=[g for g in a['grid_metrics'] if lo<=g['t']<=hi]
        xy=inv([g['pose'][:2] for g in track],p);ax.plot(xy[:,0],xy[:,1],c='#d69a15',lw=2.5,label='Measured pose trace')
        chosen=[]
        for t in [lo+1,hi-5,hi-3.5]:
            q=min(paths,key=lambda z:abs(z['t']-t));chosen.append(q)
        for j,q in enumerate(chosen):
            xy=inv(np.asarray(q['geometry'])[:,:2],p)
            ax.plot(xy[:,0],xy[:,1],c=['#93cc8c','#43a16a','#146540'][j],lw=1.8,label=f"Local path {q['t']:.2f}s")
        corners=np.array([[.36,.25],[.36,-.25],[-.36,-.25],[-.36,.25]])
        ax.add_patch(Polygon(corners,fill=False,edgecolor='#be4451',lw=2,label='Body at stop'))
        ax.set(title=title,xlabel='Forward from stop pose (m)',ylabel='Left from stop pose (m)',xlim=(-3.5,3.8),ylim=(-2,2))
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='upper left')
    fig.suptitle('Recorded failure formation: path feasibility is not execution clearance',fontsize=14)
    fig.savefig(r/'origin_geometry.png',dpi=180);fig.savefig(r/'origin_geometry.svg');plt.close(fig)
    fig,axs=plt.subplots(4,1,figsize=(13,9),sharex=True,constrained_layout=True)
    series=a['series']
    for n,c,label in [('/control/body_cmd','#427ca8','Controller'),('/cmd_vel_safe','#3e9e72','Safety output'),('/odom','#d09324','Odometry')]:
        rs=series[n];axs[0].plot([v['t'] for v in rs],[v['v'] for v in rs],c=c,label=label,lw=1)
        axs[1].plot([v['t'] for v in rs],[v['w'] for v in rs],c=c,label=label,lw=1)
    rs=series['/planning/local_replan_status'];axs[2].step([v['t'] for v in rs],[int(v.get('stop_requested',False)) for v in rs],where='post',c='#be4451')
    gs=a['grid_metrics'];axs[3].plot([g['t'] for g in gs],[g['rectangle_to_raw_grid_gap_m'] for g in gs],c='#8052a0')
    for ax,y in zip(axs,['Speed (m/s)','Yaw rate (rad/s)','Planner stop','Body to raw-cell gap (m)']):
        ax.set_ylabel(y);ax.grid(alpha=.25);ax.set_xlim(164,184);ax.axvline(176.91,c='#be4451',ls='--',lw=1)
    axs[0].legend(loc='upper right');axs[-1].set_xlabel('Seconds from bag start')
    fig.suptitle('Second long stop: controller, Safety and odometry')
    fig.savefig(r/'origin_timeline.png',dpi=180);fig.savefig(r/'origin_timeline.svg');plt.close(fig)
    if (r/'fixed_path_following.json').exists():
        fixture=min(json.loads((r/'origin_fixtures.json').read_text()),key=lambda f:abs(f['t']-171.327))
        goal=min(json.loads((r/'probe_forward_goal_only.json').read_text())['results'],key=lambda f:abs(f['t']-fixture['t']))
        model=min(json.loads((r/'fixed_path_following.json').read_text())['results'],key=lambda f:abs(f['fixture_t']-fixture['t']))
        fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True);p=fixture['pose']
        xy=inv(fixture['full_points'],p);ax.scatter(xy[:,0],xy[:,1],s=6,color='#dedee6',label='All recorded inflated cells')
        xy=inv(np.asarray(fixture['published_path'])[:,:2],p);ax.plot(xy[:,0],xy[:,1],c='#3274ad',lw=2.5,label='Recorded short rejoin (before obstacle)')
        xy=inv([[v['x'],v['y']] for v in goal['path']],p);ax.plot(xy[:,0],xy[:,1],c='#24854b',lw=2,label='Goal beyond obstacle: geometrically clear')
        xy=inv([[v['x'],v['y']] for v in model['samples']],p);ax.plot(xy[:,0],xy[:,1],c='#bf4551',lw=1.6,ls='--',label='Ideal MPPI/Safety model; envelope violation')
        ax.scatter(*xy[-1],c='#bf4551',marker='x',s=80,zorder=6)
        ax.set(xlim=(-.5,5.2),ylim=(-1.7,1.7),xlabel='Forward from initial pose (m)',ylabel='Left from initial pose (m)',title='171.327 s input: finding a path does not establish tracking clearance')
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower left')
        fig.savefig(r/'origin_goal_and_margin.png',dpi=180);fig.savefig(r/'origin_goal_and_margin.svg');plt.close(fig)


if __name__=='__main__':
    main()
