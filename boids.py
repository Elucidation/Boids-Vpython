from visual import *
from random import random

# Scene box size (LxLxL) 
L = 100 # meters
dt = 0.1 # Timestep
win= 600 # Actual box window size WxH in pixels

# Number of boids
N = 100 
RangeOfView = 40 # m, how far a bird can see
MinDist = 15 # m, separation distance
VelMax = 15 # Max velocity of bird
Radius = 0.5 # Bird radius
doTrails = False

# Tweak Constants for boids simulation
k0 = 1 # original velocity
k1 = 0.07 # avoidance
k2 = 0.1 # alignment
k3 = 0.01 # cohesion
k4 = 0.003 # global goal

# Colors to use to show boids
colors = [color.red, color.green, color.blue,
          color.yellow, color.cyan, color.magenta]



scene = display(title="Boids", width=win, height=win,
                range=2*L, forward=(-1,-1,-1))

xaxis = curve(pos=[(0,0,0), (L,0,0)], color=(0.5,0.5,0.5))
yaxis = curve(pos=[(0,0,0), (0,L,0)], color=(0.5,0.5,0.5))
zaxis = curve(pos=[(0,0,0), (0,0,L)], color=(0.5,0.5,0.5))

Boids = []
plist = [] # Positions
vlist = [] # Velocities



for i in range(N):
  x = -L+2*L*random()
  y = -L+2*L*random()
  z = -L+2*L*random()
  r = Radius
  Boids = Boids+[sphere(pos=(x,y,z), radius=r, color=colors[i % 6],
                   make_trail=doTrails, interval=10)]

  vx = -VelMax + 2*VelMax*random()
  vy = -VelMax + 2*VelMax*random()
  vz = -VelMax + 2*VelMax*random()

  plist.append( (x,y,z) )
  vlist.append( (vx,vy,vz) )

p = array(plist)
v = array(vlist)

while True:
  rate(100)


  r = p - p[:,newaxis]
  rmag = sqrt(sum(square(r),-1)) # scalar dists between boids

  

  for i in range(N):
    # Seperation (avoid close)
    hit = less_equal(rmag[i],MinDist)
    hit[i] = 0 # no self collision

    va = array( (0,0,0) )
    for b in p[hit]:
      # For too close neighbors
      va = va + 1.0/(p[i]-b)

    # Alignment (steer to average heading)
    nearby = less_equal(rmag[i],RangeOfView)
    vb = average(v[nearby], 0)      

    # Cohesion (move to average pos of local)
    centerOfMass = average(p[nearby], 0)
    vc = centerOfMass - p[i]

    # Center of screen
    vd = -p[i]

    # Update
    v[i] = k0*v[i] + k1*va + k2*vb + k3*vc + k4*vd # Magic happens here
    #v[i] = average([v[i], va, vb,vc])
    
    if mag(v[i]) > VelMax:
      v[i] = v[i] / mag(v[i]) * VelMax
    
  for i in range(N):
    p[i] = p[i] + v[i] * dt
    Boids[i].pos = p[i]

#  p = p + v*dt
