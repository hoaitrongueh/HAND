
def smoothstep(u):
    u=max(0,min(u,1))
    s=10*(u**3) - 15*(u**4) + 6*(u**5)
    return s

def interpolate(start,goal,progress):
    s=smoothstep(progress)
    position=start+s*(goal-start)
    return position
