from madcad import *
from madcad.joints import *
from madcad.kinematic.assembly import *

from foot_function import foot
from strainwave import strainwave

convex = convexhull(web([
	Circle(Axis(O,Z), 0.8),
	Circle(Axis(0, X), 0.5).mesh().transform(scaledir(Z, 0.75)).transform(X+0.5*Z),
	Circle(Axis(0.5*Z, normalize(2*Z-X)), 0.8),
	]))
filet(convex, convex.group(2).outlines(), depth=0.05)
filet(convex, convex.group(0).outlines(), depth=0.05)


a = 90
l = 400
e = 100
w = 0
c = 50
name = 'leg'
joints = [
	Revolute(('chest', 'shoulder'), Axis(-a*X, X)),
	Revolute(('shoulder', 'backleg'), shoulder := Axis(O, Y)),
	Revolute(('backleg', 'backedge'), backknee_top := Axis(-(l/2-e)*Z, Y)),
	Revolute(('backleg', 'foreedge'), backknee_bot := Axis(-l/2*Z, Y)),
	Revolute(('backedge', 'foreleg'), foreknee_top := Axis(-(l/2-e)*Z-l/2*X - w*Y, Y)),
	Revolute(('foreedge', 'foreleg'), foreknee_bot := Axis(-l/2*Z-l/2*X - w*Y, Y)),
	Revolute(('backedge', 'traverse'), Axis(-(l/2-e)*Z-e*(X-0.5*Z) - w*Y, Y)),
	Revolute(('foreedge', 'traverse'), Axis(-(l/2)*Z-e*(X-0.5*Z) - w*Y, Y)),
	Ball(('foreleg', 'foot'), foot_center := -l*Z-l/2*X-c*Z),
	]
foot_center = -l*Z-l/2*X-c*Z

foot1 = foot(fore_length=100, side_length=80, parallelogram_height=50, width=10)
kin = Kinematic(joints, ground='chest', content={
	'chest': strainwave(50, 60).place((Revolute, Axis(O,Z), Axis(-a*X, -X))),
	'shoulder': strainwave(50, 60).place((Revolute, Axis(O,Z), Axis(a*Y, Y))),
	'backleg': strainwave(50, 60).place((Revolute, Axis(O,Z), Axis(a*0.7*Y - l/2*Z + e*Z, Y))),
	'foreleg': foot1.transform(placement((Revolute, Axis(O,Z), Axis(foot_center, Z))) * rotate(pi, Z)),
	})
