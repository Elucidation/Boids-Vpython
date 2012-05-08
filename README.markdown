VPython Boids implementation
---
A basic [boids](http://www.red3d.com/cwr/boids/) implementation using the 3 main rules of Boids & 1 global rule

![Screenshot closeup](http://i.imgur.com/rUhgA.png)

* Seperation
va = sum of inverse distances to neighbors that are too close

* Alignment
vb = average of neighbor velocities

* Cohesion
vc = velocity towards center of mass of neighborhood 

* Global goal to origin
vd = velocity towards (0,0,0) origin of system

Combination of all rules with tweakable weights
```
    v[i] = k0*v[i] + k1*va + k2*vb + k3*vc + k4*vd
```


![Screenshot](http://i.imgur.com/NQqwM.png)
