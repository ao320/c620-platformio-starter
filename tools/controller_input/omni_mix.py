"""120-degree omni mixer. Body +x points toward wheel ID1; +y is left.
Viewed from above, wheel IDs 1,2,3 are CLOCKWISE. Positive rolling directions
are assumed tangential CCW and must be verified with lifted wheels.
"""
import math
MAX_RPM=900
DEADZONE=0.12

def deadzone(x,y):
    x=max(-1.0,min(1.0,float(x)));y=max(-1.0,min(1.0,float(y)))
    length=math.hypot(x,y)
    if length<=DEADZONE:return 0.0,0.0
    scaled=(min(length,1.0)-DEADZONE)/(1-DEADZONE)
    return x/length*scaled,y/length*scaled

def mix(axes,signs=(1,1,1)):
    if len(axes)<3 or len(signs)!=3 or any(s not in (-1,1) for s in signs):raise ValueError('invalid_input')
    if not all(math.isfinite(float(v)) for v in axes[:3]):raise ValueError('invalid_input')
    x,y=deadzone(axes[0],axes[1])
    _,turn_axis=deadzone(0,axes[2])
    forward,left,turn=-y,x,-turn_axis
    angles=(0,-2*math.pi/3,-4*math.pi/3)
    wheels=[-math.sin(a)*forward+math.cos(a)*left+turn for a in angles]
    scale=max(1.0,*(abs(w) for w in wheels))
    targets=[int(round(w/scale*MAX_RPM))*s for w,s in zip(wheels,signs)]
    return dict(targets=targets,forward=forward,left=left,turn=turn,neutral=all(t==0 for t in targets))
