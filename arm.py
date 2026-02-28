from copy import copy

from madcad import *
from madcad.joints import *
from madcad.assembly import placement
from pnprint import nprint

from joint import joint_innermotor
from gearbox.strainwave import strainwave_dual_crown, circular_screwing

#def kinematics(kinematics: dict[str, Kinematic]) -> Kinematic:
#	joints = []
#	for name, kinematic in kinematics.items():
#		for joint in kinematic:
#			new = copy(joint)
#			new.solids = tuple('{}_{}'.format(
#
#
#shoulder = O
#def segment(base:str, tip:str) -> list:
#	return [
#		Revolute((base, 'back-'tip), Axis(shoulder, Z)),
#		Revolute(('back-'tip, tip), Axis(mix(
#	]

def arm(backarm:float, forearm:float):
	r = stceil(backarm * 0.3)  # typical gearbox radius
	s = stceil(backarm * 0.1)  # shift of orthogonal gearboxes
	e = 0.1  # excentricity to avoid singularities

	shoulder = O
	scapula = shoulder - r*Z -s*Y
	elbow = shoulder + backarm*Z + backarm*e*X
	wrist = elbow + forearm*Z + forearm*e*X
	tool = wrist + r*Z + forearm*e*X
	midback = mix(shoulder, elbow, 0.6) -s*Y
	midfore = mix(elbow, wrist, 0.6) -s*Y
	
	joints = Solid(
		base = copy(strainwave_dual_crown(
			rext = 70,
			nscrews = 12,
			nteeth = 60,
			guided = True,
			)),
		shoulder = copy(joint_innermotor(
			rext = 50, 
			motor_length=73,
			)),
		midback = copy(strainwave_dual_crown(
			rext = 50,
			nteeth = 60,
			guided = True,
			)),
		elbow = copy(joint_innermotor(
			rext = 50, 
			motor_length=73,
			)),
		midfore = copy(strainwave_dual_crown(
			rext = 50,
			nteeth = 60,
			guided = True,
			)),
		wrist = copy(joint_innermotor(
			rext = 50, 
			motor_length=43,
			)),
		)
	joints.base.pose = placement((Revolute, joints.base.output.perimeter.axis, Axis(scapula,-Z)))
	joints.midback.pose = placement((Revolute, joints.midback.output.perimeter.axis, Axis(midback,-Z)))
	joints.midfore.pose = placement((Revolute, joints.midfore.output.perimeter.axis, Axis(midfore,-Z)))

	joints.shoulder.pose = placement((Revolute, joints.shoulder.gearbox.output.perimeter.axis, Axis(shoulder,Y)))
	joints.elbow.pose = placement((Revolute, joints.elbow.gearbox.output.perimeter.axis, Axis(elbow,Y)))
	joints.wrist.pose = placement((Revolute, joints.wrist.gearbox.output.perimeter.axis, Axis(wrist,Y)))

	def link_inner(base, tip):
#		body = convexhull(mesh.mesh([
#			inflate(extrusion(base.perimeter, - base.dscrew * base.perimeter.axis.direction).transform(base.pose), - base.dscrew*1.5),
#			inflate(extrusion(tip.perimeter, - tip.dscrew * tip.perimeter.axis.direction).transform(tip.pose), - tip.dscrew*1.5),
#			]))
		body = convexhull(mesh.mesh([
			icosphere(base.perimeter.center, base.perimeter.radius + base.dscrew*2.5).transform(base.pose),
			icosphere(tip.perimeter.center, tip.perimeter.radius + tip.dscrew*2).transform(tip.pose),
			]))
		slot_tip = revolution(
			wire([
				tip.perimeter.radius*X - tip.dscrew*1*X - tip.perimeter.radius*3*Z,
				tip.perimeter.radius*X - tip.dscrew*1*X,
				tip.perimeter.radius*X + tip.dscrew*5*X,
				tip.perimeter.radius*X + tip.dscrew*5*X + tip.perimeter.radius*2*(X+Z),
				]).segmented(),
			axis = tip.perimeter.axis,
			).transform(tip.pose * translate(tip.perimeter.center)).finish()
		slot_base = square(base.perimeter.axis.transform(base.pose), base.perimeter.radius*3)
			
		return Solid(body=intersection(body, slot_tip + slot_base))

	def link_outer(base, tip):
		body = convexhull(mesh.mesh([
			icosphere(base.perimeter.center, base.perimeter.radius + base.dscrew*1.5).transform(base.pose),
			icosphere(tip.perimeter.center, tip.perimeter.radius + tip.dscrew*1.2).transform(tip.pose),
			]))
		profile_inner = Softened([
			base.pose * base.perimeter.center - base.perimeter.radius*1.5*Z,
			base.pose * base.perimeter.center - base.perimeter.radius*0*Z,
			base.pose * base.perimeter.center + base.perimeter.radius*1*Z,
			tip.pose * tip.perimeter.center - tip.perimeter.radius*1.5*Y - tip.perimeter.radius*0.2*Z,
			])
		profile_outer = Softened([
			base.pose * base.perimeter.center - base.perimeter.radius*1.5*Z + base.perimeter.radius*0.5*Y,
			base.pose * base.perimeter.center + base.perimeter.radius*1*Z + base.perimeter.radius*0.5*Y,
			tip.pose * tip.perimeter.center + tip.perimeter.radius*1.5*Y - tip.perimeter.radius*0.2*Z,
			])
		body = intersection(body, 
			extrusion(profile_inner, max(tip.perimeter.radius, base.perimeter.radius)*3*X, alignment=0.5).flip()
			+ extrusion(profile_outer, max(tip.perimeter.radius, base.perimeter.radius)*3*X, alignment=0.5)
			)
		slot_base = revolution(
			wire([
				1 - base.perimeter.radius*3*Z,
				base.perimeter.radius*X + base.dscrew*1.6*X - base.perimeter.radius*Z,
				base.perimeter.radius*X + base.dscrew*1.6*X,
				base.perimeter.radius*X - base.dscrew*1*X,
				base.perimeter.radius*X - base.dscrew*1*X + base.perimeter.radius*2*Z,
			]).segmented().flip(),
			axis = base.perimeter.axis,
			).transform(base.pose * translate(base.perimeter.center)).finish()
		slot_tip = revolution(
			wire([
				tip.perimeter.radius*X + tip.dscrew*2*X - tip.perimeter.radius*Z,
				tip.perimeter.radius*X + tip.dscrew*2*X,
				tip.perimeter.radius*X - tip.dscrew*1*X,
				tip.perimeter.radius*X - tip.dscrew*1*X + tip.perimeter.radius*0.6*Z,
				tip.perimeter.radius*4*Z,
			]).segmented().flip(),
			axis = tip.perimeter.axis,
			).transform(tip.pose * translate(tip.perimeter.center)).finish()
		body = intersection(body, intersection(slot_base, slot_tip))
#		filet(body, body.frontiers(3,2) + body.frontiers(3,0), width=base.perimeter.radius*0.1)
		return Solid(body=body)

	return Kinematic([
		Revolute(('base', 'shoulder'), Axis(scapula,Z)),
		Revolute(('shoulder', 'back_back'), Axis(shoulder,Y)),
		Revolute(('back_back', 'back_fore'), Axis(midback,Z)),
		Revolute(('back_fore', 'fore_back'), Axis(elbow,Y)),
		Revolute(('fore_back', 'fore_fore'), Axis(midfore,Z)),
		Revolute(('fore_fore', 'wrist'), Axis(wrist,Y)),
		Revolute(('wrist', 'tool'), Axis(tool,Z)),
	], content={
		'base': Solid(joint = joints.base),
		'shoulder': Solid(
			joint = joints.shoulder, 
			body = link_inner(
				joints.base.deloc('output'), 
				joints.shoulder.deloc('gearbox', 'output'),
				),
			),
		'back_back': Solid(
			joint = joints.midback,
			body = link_outer(
				joints.shoulder.deloc('gearbox', 'shell', 'output'), 
				joints.midback.deloc('shell', 'output'),
				),
			),
		'back_fore': Solid(
			joint = joints.elbow,
			body = link_inner(
				joints.midback.deloc('output'), 
				joints.elbow.deloc('gearbox', 'output'),
				),
			),
		'fore_back': Solid(
			joint = joints.midfore,
			body = link_outer(
				joints.elbow.deloc('gearbox', 'shell', 'output'), 
				joints.midfore.deloc('shell', 'output'),
				),
			),
		'fore_fore': Solid(
			joint = joints.wrist,
			body = link_inner(
				joints.midfore.deloc('output'), 
				joints.wrist.deloc('gearbox', 'output'),
				),
			),
	})


if __name__ == '__madcad__':
	settings.resolution = ('sqradm', 1.)
	
	a = arm(200, 200)