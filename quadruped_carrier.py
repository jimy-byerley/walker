from madcad import *
from madcad.joints import *

#leg = Chain([
#	Revolute((0,1), Axis(O,normalize(2*Y-Z))),
#	Revolute((1,2), Axis(-Z,normalize(2*Y+Z))),
#	Revolute((2,3), Axis(-3*Z,Y)),
#	])

m = mat4()

l = 100
e = 20
a = 20
kin = Kinematic([
#	Revolute((-1,0), Axis(a*X, Z)),
	Revolute((-1,0), Axis(a*X, X)),
	Revolute((0,1), Axis(O, Y)),
	Revolute((1,2), Axis(-(l/2-e)*Z, Y)),
	Revolute((1,3), Axis(-l/2*Z, Y)),
	Revolute((2,4), Axis(-(l/2-e)*Z+l/2*X, Y)),
	Revolute((3,4), Axis(-l/2*Z+l/2*X, Y)),
	Ball((4,5), -l*Z+l/2*X),
	], ground=-1)

w = 20
h = 30
joints = []
places = {
	'front-right': Axis(h*X+w*Y, normalize(Z)),
	'back-right': Axis(-h*X+w*Y, normalize(Z)),
	'front-left': Axis(h*X-w*Y, normalize(Z)),
	'back-left': Axis(-h*X-w*Y, normalize(Z)),
	}
for name, axis in places.items():
	joints.extend([
		Revolute(('chest', name+'-shoulder'), axis.transform(a*axis.direction), Axis(a*Z, Z)),
		Revolute((name+'-shoulder', name+'-backleg'), Axis(O, Y)),
		Revolute((name+'-backleg', name+'-backedge'), Axis(-(l/2-e)*Z, Y)),
		Revolute((name+'-backleg', name+'-foreedge'), Axis(-l/2*Z, Y)),
		Revolute((name+'-backedge', name+'-foreleg'), Axis(-(l/2-e)*Z+l/2*X, Y)),
		Revolute((name+'-foreedge', name+'-foreleg'), Axis(-l/2*Z+l/2*X, Y)),
		Revolute((name+'-backedge', name+'-traverse'), Axis(-(l/2-e)*Z+e*(0.5*Z+X), Y)),
		Revolute((name+'-foreedge', name+'-traverse'), Axis(-(l/2)*Z+e*(0.5*Z+X), Y)),
		Ball((name+'-foreleg', name+'-foot'), -l*Z+l/2*X),
		])
kin = Kinematic(joints)

#legs = [
#	Solid(kin=kin).transform(translate(axis.origin) 
##		* mat4(quat(X, axis.direction))
#		)
#	for axis in places.values()
#	]
