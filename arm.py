from copy import copy

from madcad import *
from madcad.joints import *
from madcad.assembly import placement
from pnprint import nprint

# TODO remove this ugly path
import sys
import os
sys.path.append(os.path.abspath(__file__+'/..'))


from joint import joint_innermotor
from gearbox.strainwave import strainwave_dual_crown, circular_screwing


def screw_slot(axis: Axis, dscrew: float, rslot=None, hole=0., screw=0., expand=True, flat=False) -> Mesh:
	''' slot shape for a screw
		the result can then be used in a boolean operation to reserve set a screw place in an arbitrary shape
		
		Parameters:
			axis:  the screw axis placement, z toward the screw head (part exterior)
			dscrew: the screw diameter
			rslot:  the screw head slot radius
			hole:   
				- if `True`, enables a cylindric hole for screw body of length `dscrew*3`
				- if `float`, it is the screw hole length
			screw:  if non zero, this is the length of a thiner portion of hole after `hole`, the diameter is adjusted so that the screw can screw in
			expand: 
				- if `True`, enables slots sides
				- if `float`, it is the slot sides height
			flat:   if True, the slot will be conic do receive a flat head screw
	'''
	if not rslot:	rslot = 1.1*dscrew
	o = axis[0]
	x,y,z = dirbase(axis[1])

	profile = []
	if expand:
		if isinstance(expand, bool):		expand = 2*rslot
		profile.append(ArcCentered(
			Axis(o +expand*z, y), 
			o + expand*z +rslot*z,
			o + expand*z +rslot*x,
			))
	profile.append(o + rslot*x)
	if hole or screw:
		if flat:
			profile.append(o + 0.5*dscrew*(x-z) - (rslot-dscrew)*z)
			hole = max(hole, dot(o-profile[-1], z))
		else:
			profile.append(o + 0.5*dscrew*x)
		profile.append(o + 0.5*dscrew*x - hole*z)
		profile.append(o + 0.4*dscrew*x - min(hole+0.1*dscrew, hole+screw)*z)
		profile.append(o + 0.4*dscrew*x - (hole+screw)*z)
		profile.append(o - (hole+screw+0.4*dscrew)*z)
	else:
		profile.append(o)
	return revolution(wire(profile).segmented(), Axis(o,-z)).finish()


def hemisphere(axis: Axis, radius: float) -> Mesh:
	return intersection(
		icosphere(axis.origin, radius), 
		square(axis.flip(), radius*2),
		)

def link_inner(base, tip):
	epsilon = base.perimeter.radius*1e-3
	hole_factor = 2

	# main shape determined by the interfaces to be attached
	raw = convexhull(mesh.mesh([
		hemisphere(base.perimeter.axis.flip(), base.perimeter.radius + base.dscrew*2.5).transform(base.pose),
#		hemisphere(tip.perimeter.axis.flip(), tip.perimeter.radius + tip.dscrew*3).transform(tip.pose),
		cylinder(tip.perimeter.center, tip.perimeter.axis.offset(-tip.dscrew*hole_factor*2).origin, tip.perimeter.radius + tip.dscrew*2.5).transform(tip.pose),
		])).mergegroups()
	# leave space for interfaces
	profile = wire([
			tip.pose * (tip.perimeter.center + tip.perimeter.radius*Y + tip.dscrew*0.7*Y - tip.perimeter.radius*4*Z),
			tip.pose * (tip.perimeter.center + tip.perimeter.radius*Y + tip.dscrew*0.7*Y - tip.dscrew*hole_factor*2*Z),
			tip.pose * (tip.perimeter.center + tip.perimeter.radius*Y - tip.dscrew*1*Y - tip.dscrew*hole_factor*Z),
			tip.pose * (tip.perimeter.center + tip.perimeter.radius*Y - tip.dscrew*1*Y),
			tip.pose * (tip.perimeter.center + tip.perimeter.radius*Y + tip.dscrew*6*Y),
			base.pose * (base.perimeter.center + base.perimeter.radius*Y + base.perimeter.radius*Y - base.dscrew*2*Z),
			]).segmented()
	filet(profile, [4], width=tip.dscrew*2)
	area_tip = revolution(
		profile,
		axis = tip.perimeter.axis.transform(tip.pose),
		)
	area_base = square(base.perimeter.axis.transform(base.pose), base.perimeter.radius*3)

	removal = convexhull(web([
		Circle(
			base.perimeter.axis.transform(base.pose).offset(epsilon), 
			base.perimeter.radius - base.dscrew*2),
		Circle(
			Axis(
				tip.pose * tip.perimeter.axis.offset(-base.perimeter.radius*0.6-tip.dscrew*hole_factor).origin
				- tip.perimeter.radius*0.8 * base.perimeter.axis.direction, 
				base.perimeter.axis.direction), 
			base.perimeter.radius*0.4),
		])).flip()
		

	# it is sufficient to get overall shape
	body = intersection(intersection(intersection(raw, area_tip), area_base), removal)

	# screws at interface
	tip_screws_slot = repeataround(
		screw_slot(
			Axis(tip.perimeter.center + tip.perimeter.radius*Y - tip.dscrew*hole_factor*Z + epsilon*Z, -Z).transform(tip.pose),
			tip.dscrew,
			hole = tip.dscrew*hole_factor*2,
			rslot = tip.dscrew*1.6,
			expand = tip.dscrew*hole_factor*0.6,
			),
		repetitions = tip.div,
		axis = tip.perimeter.axis.transform(tip.pose),
		)

	# base interface requires more care because screws covered by area for tip needs different integration to not reduce rigidity
	axis = base.perimeter.axis.transform(base.pose)
	plane = tip.perimeter.axis.transform(tip.pose)
	x = cross(axis.direction, plane.direction)

	# screws accessed laterally when not covered by tip
	slot_lateral = convexhull(mesh.mesh([
			screw_slot(
				Axis(base.perimeter.center - base.dscrew*hole_factor*Z + epsilon*Z, -Z).transform(base.pose).transform(base.perimeter.radius*x),
				base.dscrew,
				rslot = base.dscrew*1.6,
				expand = base.dscrew*hole_factor,
				),
			icosphere(base.pose * (base.perimeter.center - base.perimeter.radius*2*x - base.dscrew*hole_factor*3*Z), base.dscrew*2),
			])).mergegroups().flip()
	# screws accessed axially when covered by tip
	slot_axial = screw_slot(
				Axis(base.perimeter.center - base.dscrew*hole_factor*Z + epsilon*Z, -Z).transform(base.pose).transform(base.perimeter.radius*x),
				base.dscrew,
				rslot = base.dscrew*1.6,
				expand = base.dscrew*hole_factor*2,
				)

	#  gather all base screw slots
	slots = []
	points = []
	for a in linrange(0, 2*pi, step=2*pi/base.div, end=False):
#		t = rotatearound(a, axis)
		t = translate(axis.origin) * rotate(a, axis.direction) * translate(-axis.origin)
		p = t * (axis.origin + base.perimeter.radius * x)
		proj = dot(p - plane.origin, plane.direction)
		points.append(t)
		if proj < 0:
			slots.append(slot_lateral.transform(t))
		else:
			slots.append(slot_axial.transform(t))
	# add screw holes
	holes = repeataround(cylinder(
		base.perimeter.center + base.perimeter.radius*Y - base.dscrew*(hole_factor+1)*Z,
		base.perimeter.center + base.perimeter.radius*Y + epsilon*Z,
		base.dscrew*0.5,
		), 
		axis = base.perimeter.axis, 
		repetitions = base.div,
		).transform(base.pose).flip()

	base_screws_slot = intersection(holes, mesh.mesh(slots))

	body = intersection(body, tip_screws_slot + base_screws_slot)

	return Solid(
		body = body.finish().option(color=vec3(0.4, 0.1, 0.05)),
		screws = Solid(),
		)

def link_outer(base, tip):
	epsilon = base.perimeter.radius*1e-3
	body1 = convexhull(mesh.mesh([
		hemisphere(base.perimeter.axis.flip(), base.perimeter.radius + base.dscrew*1.5).transform(base.pose),
		hemisphere(tip.perimeter.axis.flip(), tip.perimeter.radius + tip.dscrew*1.2).transform(tip.pose),
		])).mergegroups()
	profile_inner = Softened([
		base.pose * (base.perimeter.center + base.perimeter.radius*1.5*Y),
		base.pose * (base.perimeter.center + base.perimeter.radius*0*Y),
		base.pose * (base.perimeter.center - base.perimeter.radius*1*Y),
		tip.pose * (tip.perimeter.center - tip.perimeter.radius*1.5*Y + tip.perimeter.radius*0.2*Z),
		])
	profile_outer = Softened([
		base.pose * base.perimeter.center - base.perimeter.radius*1.5*Z + base.perimeter.radius*0.5*Y,
		base.pose * base.perimeter.center + base.perimeter.radius*1*Z + base.perimeter.radius*0.5*Y,
		tip.pose * tip.perimeter.center + tip.perimeter.radius*1.5*Y - tip.perimeter.radius*0.2*Z,
		])
	body2 = intersection(body1, 
		extrusion(profile_inner, max(tip.perimeter.radius, base.perimeter.radius)*3*X, alignment=0.5).flip()
		+ extrusion(profile_outer, max(tip.perimeter.radius, base.perimeter.radius)*3*X, alignment=0.5)
		)
	profile = wire([
			base.perimeter.radius*0.1*X - base.perimeter.radius*3*Z,
			base.perimeter.radius*X + base.dscrew*1.4*X - base.perimeter.radius*Z,
			base.perimeter.radius*X + base.dscrew*1.4*X + epsilon*Z,
			base.perimeter.radius*X - base.dscrew*1.2*X + epsilon*Z,
			base.perimeter.radius*X - base.dscrew*1.2*X + base.perimeter.radius*2*Z,
		]).segmented()
	chamfer(profile, [3], width=tip.dscrew*0.5)
	slot_base = revolution(
		profile.flip(),
		axis = base.perimeter.axis,
		).transform(base.pose * translate(base.perimeter.center)).finish()
	profile = wire([
			tip.perimeter.radius*X + tip.dscrew*2*X - tip.perimeter.radius*Z,
			tip.perimeter.radius*X + tip.dscrew*2*X + epsilon*Z,
			tip.perimeter.radius*X - tip.dscrew*1.2*X + epsilon*Z,
			tip.perimeter.radius*X - tip.dscrew*1.2*X + tip.perimeter.radius*0.6*Z,
			tip.perimeter.radius*4*Z,
		]).segmented()
	chamfer(profile, [2], width=tip.dscrew*0.5)
	slot_tip = revolution(profile.flip(),
		axis = tip.perimeter.axis,
		).transform(tip.pose * translate(tip.perimeter.center)).finish()
	body3 = intersection(body2, intersection(slot_base, slot_tip))

	base_screw_slots = repeataround(
		cylinder(
			base.perimeter.center + base.perimeter.radius*X -epsilon*Z, 
			base.perimeter.center + base.perimeter.radius*X +base.dscrew*2.5*Z,
			base.dscrew*0.8,
			),
		axis = base.perimeter.axis,
		repetitions = base.div,
		).transform(base.pose).flip()
	tip_screw_slots = repeataround(
		cylinder(
			tip.perimeter.center + tip.perimeter.radius*X -epsilon*Z, 
			tip.perimeter.center + tip.perimeter.radius*X +tip.dscrew*2.5*Z,
			tip.dscrew*0.8,
			),
		axis = tip.perimeter.axis,
		repetitions = tip.div,
		).transform(tip.pose).flip()
	body = intersection(body3, base_screw_slots + tip_screw_slots)
	
#		filet(body, body.frontiers(3,2) + body.frontiers(3,0), width=base.perimeter.radius*0.1)
	return Solid(body=body.finish().option(color=vec3(0.4, 0.2, 0)))


def arm(backarm:float, forearm:float):
	# select the actuators we want to create the structure around
	actuators = Solid(
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
		tool = copy(joint_innermotor(
			rext = 50, 
			motor_length=43,
			)),
		)

	# reference positions
	s = stceil(backarm * 0.1)  # shift of orthogonal gearboxes
#	e = -0.1  # excentricity to avoid singularities
	e = 0
	d = backarm*0.1
#	e = 0
	shoulder = O
	scapula = shoulder - actuators.shoulder.gearbox.rext*1.5*Z -s*Y -d*0.5*X
	elbow = shoulder + backarm*Z + backarm*e*X
	wrist = elbow + forearm*Z + forearm*e*X
	tool = wrist + actuators.wrist.gearbox.rext*2.2*Z - forearm*e*X
	midback = mix(shoulder, elbow, 0.65) -s*Y +d*X
	midfore = mix(elbow, wrist, 0.65) -s*Y +d*X

	# place actuators
	actuators.base.pose = placement((Revolute, actuators.base.output.perimeter.axis, Axis(scapula,-Z)))
	actuators.midback.pose = placement((Revolute, actuators.midback.output.perimeter.axis, Axis(midback,-Z)))
	actuators.midfore.pose = placement((Revolute, actuators.midfore.output.perimeter.axis, Axis(midfore,-Z)))

	actuators.shoulder.pose = placement((Revolute, actuators.shoulder.gearbox.output.perimeter.axis, Axis(shoulder,Y)))
	actuators.elbow.pose = placement((Revolute, actuators.elbow.gearbox.output.perimeter.axis, Axis(elbow,Y)))
	actuators.wrist.pose = placement((Revolute, actuators.wrist.gearbox.output.perimeter.axis, Axis(wrist,Y)))
	actuators.tool.pose = placement((Revolute, actuators.tool.gearbox.output.perimeter.axis, Axis(tool,-Z)))

	kinematic = Kinematic([
		Revolute(('base', 'shoulder'), Axis(scapula,Z)),
		Revolute(('shoulder', 'back_back'), Axis(shoulder,Y)),
		Revolute(('back_back', 'back_fore'), Axis(midback,Z)),
		Revolute(('back_fore', 'fore_back'), Axis(elbow,Y)),
		Revolute(('fore_back', 'fore_fore'), Axis(midfore,Z)),
		Revolute(('fore_fore', 'wrist'), Axis(wrist,Y)),
		Revolute(('wrist', 'tool'), Axis(tool,Z)),
	], content={
		'base': Solid(joint = actuators.base),
		'shoulder': Solid(
			joint = actuators.shoulder, 
			body = link_inner(
				actuators.base.deloc('output'), 
				actuators.shoulder.deloc('gearbox', 'output'),
				),
			),
		'back_back': Solid(
			joint = actuators.midback,
			body = link_outer(
				actuators.shoulder.deloc('gearbox', 'shell', 'output'), 
				actuators.midback.deloc('shell', 'output'),
				),
			),
		'back_fore': Solid(
			joint = actuators.elbow,
			body = link_inner(
				actuators.midback.deloc('output'), 
				actuators.elbow.deloc('gearbox', 'output'),
				),
			),
		'fore_back': Solid(
			joint = actuators.midfore,
			body = link_outer(
				actuators.elbow.deloc('gearbox', 'shell', 'output'), 
				actuators.midfore.deloc('shell', 'output'),
				),
			),
		'fore_fore': Solid(
			joint = actuators.wrist,
			body = link_inner(
				actuators.midfore.deloc('output'), 
				actuators.wrist.deloc('gearbox', 'output'),
				),
			),
		'wrist': Solid(
			joint = actuators.tool,
			body = link_outer(
				actuators.wrist.deloc('gearbox', 'shell', 'output'),
				actuators.tool.deloc('gearbox', 'shell', 'output'), 
				),
			),
	})
	return kinematic


def arm_alternate(backarm:float, forearm:float):
	# select the actuators we want to create the structure around
	actuators = Solid(
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
		tool = copy(joint_innermotor(
			rext = 50, 
			motor_length=43,
			)),
		)

	# reference positions
	s = stceil(backarm * 0.2)  # shift of orthogonal gearboxes
	r = backarm*0.3
#	e = -0.1  # excentricity to avoid singularities
	e = 0
	shoulder = O
	scapula = shoulder - actuators.shoulder.gearbox.rext*1.5*Z
	elbow = shoulder + backarm*Z + backarm*e*X
	wrist = elbow + forearm*Z + forearm*e*X
	tool = wrist + actuators.wrist.gearbox.rext*2.2*Z - forearm*e*X -s*Y
	midback = elbow -r*Z -s*Y
	midfore = wrist -r*Z +s*Y

	# place actuators
	actuators.base.pose = placement((Revolute, actuators.base.output.perimeter.axis, Axis(scapula,-Z)))
	actuators.midback.pose = placement((Revolute, actuators.midback.output.perimeter.axis, Axis(midback,-Z)))
	actuators.midfore.pose = placement((Revolute, actuators.midfore.output.perimeter.axis, Axis(midfore,-Z)))

	actuators.shoulder.pose = placement((Revolute, actuators.shoulder.gearbox.output.perimeter.axis, Axis(shoulder,Y)))
	actuators.elbow.pose = placement((Revolute, actuators.elbow.gearbox.output.perimeter.axis, Axis(elbow,-Y)))
	actuators.wrist.pose = placement((Revolute, actuators.wrist.gearbox.output.perimeter.axis, Axis(wrist,Y)))
	actuators.tool.pose = placement((Revolute, actuators.tool.gearbox.output.perimeter.axis, Axis(tool,-Z)))

	kinematic = Kinematic([
		Revolute(('base', 'shoulder'), Axis(scapula,Z)),
		Revolute(('shoulder', 'back_back'), Axis(shoulder,Y)),
		Revolute(('back_back', 'back_fore'), Axis(midback,Z)),
		Revolute(('back_fore', 'fore_back'), Axis(elbow,Y)),
		Revolute(('fore_back', 'fore_fore'), Axis(midfore,Z)),
		Revolute(('fore_fore', 'wrist'), Axis(wrist,Y)),
		Revolute(('wrist', 'tool'), Axis(tool,Z)),
	], content={
		'base': Solid(joint = actuators.base),
		'shoulder': Solid(
			joint = actuators.shoulder, 
#			body = link_inner(
#				actuators.base.deloc('output'), 
#				actuators.shoulder.deloc('gearbox', 'output'),
#				),
			),
		'back_back': Solid(
			joint = actuators.midback,
#			body = link_outer(
#				actuators.shoulder.deloc('gearbox', 'shell', 'output'), 
#				actuators.midback.deloc('shell', 'output'),
#				),
			),
		'back_fore': Solid(
			joint = actuators.elbow,
#			body = link_inner(
#				actuators.midback.deloc('output'), 
#				actuators.elbow.deloc('gearbox', 'output'),
#				),
			),
		'fore_back': Solid(
			joint = actuators.midfore,
#			body = link_outer(
#				actuators.elbow.deloc('gearbox', 'shell', 'output'), 
#				actuators.midfore.deloc('shell', 'output'),
#				),
			),
		'fore_fore': Solid(
			joint = actuators.wrist,
#			body = link_inner(
#				actuators.midfore.deloc('output'), 
#				actuators.wrist.deloc('gearbox', 'output'),
#				),
			),
		'wrist': Solid(
			joint = actuators.tool,
#			body = link_outer(
#				actuators.wrist.deloc('gearbox', 'shell', 'output'),
#				actuators.tool.deloc('gearbox', 'shell', 'output'), 
#				),
			),
	})
	return kinematic

if __name__ == '__madcad__':
	settings.resolution = ('sqradm', 1.)
	
	a = arm(200, 200)
#	a = arm_alternate(200, 200)
	a.default = [0, pi/3, pi/3, -pi/3, -pi/6, pi/4, 0]