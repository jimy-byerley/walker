from madcad import *
from madcad.joints import *

from foot_function import foot

convex = convexhull(web([
	Circle(Axis(O,Z), 0.8),
	Circle(Axis(0, X), 0.5).mesh().transform(scaledir(Z, 0.75)).transform(X+0.5*Z),
	Circle(Axis(0.5*Z, normalize(2*Z-X)), 0.8),
	]))
filet(convex, convex.group(2).outlines(), depth=0.05)
filet(convex, convex.group(0).outlines(), depth=0.05)


a = 100
l = 300
e = 50
name = 'leg'
joints = [
	Revolute(('chest', 'shoulder'), Axis(-a*X, X)),
	Revolute(('shoulder', 'backleg'), shoulder := Axis(O, Y)),
	Revolute(('backleg', 'backedge'), backknee_top := Axis(-(l/2-e)*Z, Y)),
	Revolute(('backleg', 'foreedge'), backknee_bot := Axis(-l/2*Z, Y)),
	Revolute(('backedge', 'foreleg'), foreknee_top := Axis(-(l/2-e)*Z-l/2*X, Y)),
	Revolute(('foreedge', 'foreleg'), foreknee_bot := Axis(-l/2*Z-l/2*X, Y)),
	Revolute(('backedge', 'traverse'), Axis(-(l/2-e)*Z-e*(X-0.5*Z), Y)),
	Revolute(('foreedge', 'traverse'), Axis(-(l/2)*Z-e*(X-0.5*Z), Y)),
	Ball(('foreleg', 'foot'), foot_center := -l*Z-l/2*X-40*X),
	]
foot_center = -l*Z-l/2*X

foot1 = foot()
kin = Kinematic(joints, ground='chest', content={
	'foreleg': foot1.place((Revolute, foot1['leg'], Axis(foot_center-40*X, Z))),
	})
