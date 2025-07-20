from madcad import *

convex = convexhull(web([
	Circle(Axis(O,Z), 0.8),
	Circle(Axis(0, X), 0.5).mesh().transform(scaledir(Z, 0.75)).transform(X+0.5*Z),
	Circle(Axis(0.5*Z, normalize(2*Z-X)), 0.8),
	]))
filet(convex, convex.group(2).outlines(), ('depth', 0.05))
filet(convex, convex.group(0).outlines(), ('depth', 0.05))
