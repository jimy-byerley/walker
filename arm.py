from copy import copy

from madcad import *
from madcad.joints import *
from madcad.assembly import placement
from madcad.scheme import *
from pnprint import nprint

# TODO remove this ugly path
import sys
import os
sys.path.append(os.path.abspath(__file__+'/..'))


from utils import export
from sensors import mks_dualfoc
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
	return revolution(wire(profile), Axis(o,-z)).finish()


def hemisphere(axis: Axis, radius: float) -> Mesh:
	return intersection(
		icosphere(axis.origin, radius), 
		square(axis.flip(), radius*2),
		)

def link_inner(base, tip):
	epsilon = base.perimeter.radius*1e-3
	hole_factor = 2

	# create global vectors for base and tip, they are not necessarily the same as base and tip frames but ensure they share the same x
	tip_z = vec3(tip.pose[2])
	base_z = vec3(base.pose[2])
	x = normalize(cross(tip_z, base_z))
	tip_y = cross(tip_z, x)
	base_y = cross(base_z, x)
	base_o = base.pose * base.perimeter.center
	tip_o = tip.pose * tip.perimeter.center

	# main shape determined by the interfaces to be attached
	raw = convexhull(mesh.mesh([
		hemisphere(base.perimeter.axis.flip(), base.perimeter.radius + base.dscrew*3).transform(base.pose),
#		hemisphere(tip.perimeter.axis.flip(), tip.perimeter.radius + tip.dscrew*3).transform(tip.pose),
		cylinder(tip.perimeter.center, tip.perimeter.axis.offset(-tip.dscrew*hole_factor*2.2).origin, tip.perimeter.radius + tip.dscrew*2.5).transform(tip.pose),
		])).mergegroups()
	# leave space for interfaces
	profile = wire([
			-tip.perimeter.radius*tip_y - tip.dscrew*0.7*tip_y - tip.perimeter.radius*4*tip_z,
			-tip.perimeter.radius*tip_y - tip.dscrew*0.7*tip_y - tip.dscrew*hole_factor*2*tip_z,
			-tip.perimeter.radius*tip_y + tip.dscrew*1*tip_y - tip.dscrew*hole_factor*tip_z,
			-tip.perimeter.radius*tip_y + tip.dscrew*1*tip_y,
			-tip.perimeter.radius*tip_y - tip.dscrew*6*tip_y,
			noproject(base_o - tip_o + base.perimeter.radius*base_y + base.perimeter.radius*base_y - base.dscrew*2*base_z, x),
			]).transform(translate(tip_o)).segmented()
	filet(profile, [1,4], width=tip.dscrew*2)
	area_tip = revolution(
		profile,
		axis = tip.perimeter.axis.transform(tip.pose),
		)
	area_base = square(base.perimeter.axis.transform(base.pose), base.perimeter.radius*3)

	# it is sufficient to get overall shape
	body1 = intersection(intersection(raw, area_tip), area_base)

	# space for inner motor, is there is enough space
	if base.perimeter.radius - base.dscrew < dot(tip_o - base_o, tip_z):
		bucket = wire([
			base_o + base.perimeter.radius*x - base.dscrew*x + base.dscrew*0.5*(base_z+x),
			base_o + base.perimeter.radius*x - base.dscrew*x - base.dscrew*0.5*base_z,
			base_o + base.perimeter.radius*x - base.dscrew*x - base.dscrew*(hole_factor*2.5)*base_z,
			base_o - base.dscrew*(hole_factor*4)*base_z,
			]).segmented().flip()
		filet(bucket, [2], width=base.dscrew*2)
		removal = revolution(bucket, Axis(base_o, base_z))
		body1 = intersection(body1, removal)

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
	# TODO to this by checking intersection with the frontier rather than doing trigonometry
	slots = []
	points = []
	# alignment should be pi/base.div - asin(...)
	alignment = pi/base.div - 0.2*asin(dot(
		tip.pose * tip.perimeter.center - base.pose * base.perimeter.center, 
		cross(mat3(tip.pose) * tip.perimeter.axis.direction, mat3(base.pose) * base.perimeter.axis.direction),
		)/base.perimeter.radius)
	for a in linrange(0, 2*pi, step=2*pi/base.div, end=False):
		t = translate(axis.origin) * rotate(a + alignment, axis.direction) * translate(-axis.origin)
		p = t * (axis.origin + base.perimeter.radius * x)
		proj = dot(p - plane.origin, plane.direction)
		points.append(t)
		if proj > -base.dscrew*0.2:
			slots.append(slot_axial.transform(t))
		else:
			slots.append(slot_lateral.transform(t))
	# add screw holes
	holes = repeataround(
		convexhull(mesh.mesh([
			cylinder(
				base.perimeter.center + base.perimeter.radius*Y - base.dscrew*(hole_factor+epsilon)*Z,
				base.perimeter.center + base.perimeter.radius*Y + epsilon*Z,
				base.dscrew*0.5,
				), 
			icosphere(base.perimeter.center + base.perimeter.radius*Y - base.dscrew*(hole_factor+epsilon)*Z, base.dscrew*0.5),
			fill(Circle(Axis(base.perimeter.center + base.perimeter.radius*Y - base.dscrew*hole_factor*Y, Z), base.dscrew*0.5)),
			])).mergegroups(),
		axis = base.perimeter.axis, 
		repetitions = base.div,
		).transform(rotate(alignment, Z)).transform(base.pose).flip()

	base_screws_slot = intersection(holes, mesh.mesh(slots))

	body = intersection(body1, tip_screws_slot + base_screws_slot)

	return Solid(
		body = body.finish(),
#		body = body.finish().option(color=vec3(0.4, 0.1, 0.05)),
#		body = body.finish().option(color=vec3(0.1, 0.2, 0.4)),
		screws = Solid(),
		)

def driver_box():
	epsilon = 1e-3
	driver = mks_dualfoc(controller=True)
	height = stceil(driver.width*0.5)
	prelength = stceil(driver.width*0.6)

	profile = wire(Softened([
		driver.width*0.5*X + height*0.5*Z,
		driver.width*0.7*X + height*0.5*Z,
		driver.width*0.7*X,
		driver.controller.width*0.8*X - driver.width*0.5*Z,
		driver.controller.width*0.5*X - driver.width*0.5*Z,
		])).transform(driver.pose)
	controller = driver.controller
	dscrew = driver.dscrew
	path = wire([
		controller.width*0.5*X,
		Interpolated([
			controller.width*0.5*X + controller.length*Y - dscrew*Y,
			controller.width*0.5*X + controller.length*Y - dscrew*X,
			controller.length*Y + controller.width*0.1*Y,
			-controller.width*0.5*X + controller.length*Y + dscrew*X,
			-controller.width*0.5*X + controller.length*Y - dscrew*Y,
		]),
		-controller.width*0.5*X,
		]).transform(driver.pose * translate(-driver.width*0.5*Z))
	boundary = convexhull(
		tube(profile.transform((path[0]-profile[-1])), path)
		+ hemisphere(Axis(+driver.width*0.5*X -prelength*Y,Y), height*0.5)
		+ hemisphere(Axis(-driver.width*0.5*X -prelength*Y,Y), height*0.5)
		).mergegroups()
	
	slot = screw_slot(
			Axis(O, Y).transform(driver.pose),
			dscrew,
			hole = dscrew*2,
			rslot = dscrew*2.5,
			expand = prelength - dscrew*3,			)
	screw_slots = mesh.mesh([
		slot.transform(+driver.width*0.5*X -prelength*Y +(height*0.5-dscrew)*X +dscrew*Y),
		slot.transform(-driver.width*0.5*X -prelength*Y -(height*0.5-dscrew)*X +dscrew*Y),
		])
#	boundary = intersection(boundary, screw_slots)

	thickness = 1.5
	inside = union(
		union(
			inflate(extrusion(fill(driver.outline), driver.width*Z).orient(), thickness),
			inflate(extrusion(fill(driver.controller.outline.transform(driver.controller.pose)), driver.width*Z).orient(), thickness),
			),
		inflate(convexhull(driver.controller.connections['gpios']), thickness-epsilon).transform(driver.controller.pose),
		)

	profile = wire([
		-height*2*Z + driver.length*0.3*Y,
		-height*0.5*Z +driver.length*0.3*Y,
		-height*0.5*Z -driver.width*2*Y,
		]).flip().transform(driver.pose)
	filet(profile, [1], width=height*0.4)
	profile = extrusion(profile, driver.width*2*X, alignment=0.5)

	connections = parallelogram(
		driver.controller.width*1.1*X, 
		-driver.width*2*Y, 
		driver.width*2*Z, 
		origin=driver.controller.width*0.6*Y, 
		fill=True, 
		alignment=vec3(0.5, 0, 0.5))

	body = intersection(intersection(intersection(boundary, inside.flip()), connections), profile + screw_slots)

	return Solid(
		electronics = driver,
		body = body.finish(),
		)

def link_outer(base, tip, driver=False):
	epsilon = base.perimeter.radius*5e-3

	tip_z = vec3(tip.pose[2])
	base_z = vec3(base.pose[2])
	x = normalize(cross(tip_z, base_z))
	tip_y = cross(tip_z, x)
	base_y = cross(base_z, x)
	base_o = base.pose * base.perimeter.center
	tip_o = tip.pose * tip.perimeter.center


	body1 = convexhull(mesh.mesh([
		hemisphere(base.perimeter.axis.flip(), base.perimeter.radius + base.dscrew*1.5).transform(base.pose),
		hemisphere(tip.perimeter.axis.flip(), tip.perimeter.radius + tip.dscrew*1.2).transform(tip.pose),
		])).mergegroups()
	profile_inner = Softened([
		base_o + base.perimeter.radius*1.5*base_y,
		base_o + base.perimeter.radius*0*base_y,
		base_o - base.perimeter.radius*1*base_y,
		tip_o + tip.perimeter.radius*1.5*tip_y + tip.perimeter.radius*0.2*tip_z,
		])
	profile_outer = Softened([
		base_o + base.perimeter.radius*1.5*base_y + base.perimeter.radius*0.5*base_z,
		base_o - base.perimeter.radius*1*base_y + base.perimeter.radius*0.5*base_z,
		tip_o - tip.perimeter.radius*1.5*tip_y + tip.perimeter.radius*0.2*tip_z,
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
			tip.perimeter.radius*X + tip.dscrew*3*X - tip.perimeter.radius*Z,
			tip.perimeter.radius*X + tip.dscrew*3*X + epsilon*Z,
			tip.perimeter.radius*X - tip.dscrew*1.2*X + epsilon*Z,
			tip.perimeter.radius*X - tip.dscrew*1.2*X + tip.perimeter.radius*0.6*Z,
			tip.perimeter.radius*4*Z,
		]).segmented()
	chamfer(profile, [2], width=tip.dscrew*0.5)
	slot_tip = revolution(profile.flip(),
		axis = tip.perimeter.axis,
		).transform(tip.pose * translate(tip.perimeter.center)).finish()
	body3 = intersection(body2, intersection(slot_base, slot_tip))

	def screw_slots(interface):
		return repeataround(
			cylinder(
				interface.perimeter.center + interface.perimeter.radius*X -epsilon*Z, 
				interface.perimeter.center + interface.perimeter.radius*X +interface.dscrew*2.5*Z,
				interface.dscrew*3/5,
				),
			axis = interface.perimeter.axis,
			repetitions = interface.div,
			).transform(interface.pose).flip()


	def sensor_slot(interface, depth, height):
		start = interface.perimeter.center + interface.perimeter.radius*Y - interface.dscrew*Y
		slot = (extrusion(
			fill(wire([
				start - 0.1*interface.dscrew*Z - interface.dscrew*2*Y,
				start - 0.1*interface.dscrew*Z + depth*Y,
				start + depth*Y + height*Z,
				start - interface.dscrew*2*Y + height*Z + depth*Z,
			])),
			(interface.perimeter.radius*2*sin(pi/interface.div) - interface.dscrew*2.5)*X,
			alignment = 0.5,
			).transform(
				interface.pose 
				* rotate(pi/interface.div,Z) 
			).flip())
		filet(slot, slot.frontiers(), width=base.dscrew*0.4)
		return slot
	
	base_sensor_slot = sensor_slot(base, base.dscrew*3, base.perimeter.radius*0.35)
	tip_sensor_slot = sensor_slot(tip, tip.dscrew*1.4, tip.perimeter.radius*0.7)

	body4 = intersection(body3, screw_slots(base) + screw_slots(tip))
	body = intersection(body4, union(base_sensor_slot, tip_sensor_slot).flip())

	if driver:	
		driver = mks_dualfoc(controller=True)
#		driver2 = driver.transform(
#			translate(tip.pose*tip.perimeter.center + tip.perimeter.radius*0.8*tip_z - driver.length*0.7*tip_y) 
#			* mat4(mat3(x, tip_y, tip_z)) 
#			* rotate(-pi/2,X)
#			* rotate(pi/2,Z)
#			* translate(-driver.length*0.3*Y))
#		driver2 = driver.transform(
#			translate(tip_o + tip.dscrew*4*tip_z)
#			* mat4(mat3(*dirbase(normalize(x - tip_y), normalize(tip_o - base_o))))
#			* translate(tip.perimeter.radius*1.2*Z)
#			* rotate(-pi/2,Z)
#			* translate(-driver.length*Y)
#			)
#		driver2 = driver.transform(
#			translate(tip_o + tip.dscrew*4*tip_z)
#			* mat4(mat3(*dirbase(-x, normalize(noproject(tip_o - base_o, base_z)))))
#			* translate(-tip.perimeter.radius*1.2*Z)
#			* rotate(-pi/2,Z)
#			* translate(-driver.length*Y)
#			)
#		driver = driver.transform(
#			translate(tip_o + tip.perimeter.radius*0.85*tip_z - driver.length*Y + tip.perimeter.radius*Y -tip.perimeter.radius*0.1*x) 
#			* mat4(mat3(x, tip_y, tip_z)) 
#			* rotate(pi,X))

#		driver = driver.transform(
		driver = driver_box().transform(
			translate(tip_o + tip.perimeter.radius*0.85*tip_z - tip.perimeter.radius*0.4*tip_y) 
			* mat4(mat3(x, tip_y, tip_z)) 
			* rotate(pi,X) 
			* rotate(pi,Z))
	else:
		driver = None
		
	return Solid(
		body = body.finish().option(color=vec3(0.4, 0.2, 0)),
		driver = driver,
		)


def arm_repeated(backarm:float, forearm:float):
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
		midback = copy(joint_innermotor(
			rext = 50, 
			motor_length=43,
			)),
		elbow = copy(joint_innermotor(
			rext = 50, 
			motor_length=73,
			)),
		midfore = copy(joint_innermotor(
			rext = 50, 
			motor_length=43,
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
	driver = mks_dualfoc(controller=True)

	# reference positions
	s = stceil(backarm * 0.1)  # shift of orthogonal gearboxes
#	e = -0.1  # excentricity to avoid singularities
	e = 0
	d = backarm*0.08
	u = backarm*0.1
#	e = 0
	shoulder = O
	scapula = shoulder - actuators.shoulder.rext*1.5*Z -s*Y -d*0.5*X
	elbow = shoulder + backarm*Z + backarm*e*X +u*Y
	wrist = elbow + forearm*Z + forearm*e*X +u*Y
	tool = wrist + actuators.wrist.rext*2.2*Z - forearm*e*X
	midback = elbow + normalize(shoulder - elbow) * (actuators.elbow.rext + actuators.elbow.output.dscrew*4) -s*Y +d*X
	midfore = wrist + normalize(elbow - wrist) * (actuators.wrist.rext + actuators.wrist.output.dscrew*4) -s*Y +d*X

	# place actuators
	actuators.base.pose = placement((Revolute, actuators.base.output.perimeter.axis, Axis(scapula,-Z)))
	actuators.midback.pose = placement((Revolute, actuators.midback.output.perimeter.axis, Axis(midback,-Z)))
	actuators.midfore.pose = placement((Revolute, actuators.midfore.output.perimeter.axis, Axis(midfore,-Z)))

	actuators.shoulder.pose = placement((Revolute, actuators.shoulder.output.perimeter.axis, Axis(shoulder,Y))) * rotate(pi,Z)
	actuators.elbow.pose = placement((Revolute, actuators.elbow.output.perimeter.axis, Axis(elbow,Y))) * rotate(pi,Z)
	actuators.wrist.pose = placement((Revolute, actuators.wrist.output.perimeter.axis, Axis(wrist,Y))) * rotate(pi,Z)
	actuators.tool.pose = placement((Revolute, actuators.tool.output.perimeter.axis, Axis(tool,-Z)))

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
#			driver1 = driver.transform(actuators.base.pose * translate(-110*Z - 40*Y) * rotate(pi,Z) * rotate(pi/2,X)),
			body = link_inner(
				actuators.base.deloc('output'), 
				actuators.shoulder.deloc('output'),
				),
			),
		'back_back': Solid(
			shoulder = actuators.shoulder, 
			midback = actuators.midback,
#			driver1 = driver.transform(actuators.shoulder.pose * translate(40*Z - 30*Y) * rotate(pi,Y)),
#			driver2 = driver.transform(actuators.shoulder.pose * translate(15*Z + 65*Y) * rotate(pi/2,Z) * rotate(3.5,Y) * translate(-driver.length/2*Y)),
#			driver = driver.transform(actuators.midback.pose * translate(35*Z - 10*Y) * rotate(pi,Y) * rotate(pi,Z) * translate(-driver.length*0.4*Y)),
			body = link_outer(
				actuators.shoulder.deloc('shell', 'output'), 
				actuators.midback.deloc('shell', 'output'),
				driver=True,
				),
			annotations = Solid(
				backarm = note_distance(shoulder, elbow, project=Z, offset=backarm*0.8*Y),
				midback = note_distance(midback, elbow, project=Z, offset=backarm*0.6*Y),
				u = note_distance(shoulder, elbow, project=Y, offset=-backarm*0.8*X),
				d = note_distance(midback, elbow, project=X, offset=-backarm*0.6*Y),
				),
			),
		'back_fore': Solid(
			body = link_inner(
				actuators.midback.deloc('output'), 
				actuators.elbow.deloc('output'),
				),
			),
		'fore_back': Solid(
			elbow = actuators.elbow,
			midfore = actuators.midfore,
#			driver1 = driver.transform(actuators.elbow.pose * translate(40*Z - 30*Y) * rotate(pi,Y)),
#			driver2 = driver.transform(actuators.elbow.pose * translate(15*Z - 70*Y) * rotate(pi/2,Z) * rotate(pi,Y) * translate(-driver.length/2*Y)),
#			driver = driver.transform(actuators.midfore.pose * translate(35*Z - 10*Y) * rotate(pi,Y) * rotate(pi,Z) * translate(-driver.length*0.4*Y)),
			body = link_outer(
				actuators.elbow.deloc('shell', 'output'), 
				actuators.midfore.deloc('shell', 'output'),
				driver=True,
				),
			annotations = Solid(
				backarm = note_distance(elbow, wrist, project=Z, offset=forearm*0.8*Y),
				midback = note_distance(midfore, wrist, project=Z, offset=forearm*0.6*Y),
				u = note_distance(elbow, wrist, project=Y, offset=-forearm*0.8*X),
				d = note_distance(midfore, wrist, project=X, offset=-forearm*0.6*Y),
				),
			),
		'fore_fore': Solid(
			body = link_inner(
				actuators.midfore.deloc('output'), 
				actuators.wrist.deloc('output'),
				),
			driver = driver.transform(actuators.midfore.pose 
				* translate(-120*Z - 10*Y + 22*X) 
				* rotate(pi,Z) 
				* rotate(pi/2-0.3,X)
				* rotate(-0.3,Z)),
			),
		'wrist': Solid(
			wrist = actuators.wrist,
			tool = actuators.tool,
			body = link_outer(
				actuators.wrist.deloc('shell', 'output'),
				actuators.tool.deloc('shell', 'output'), 
				),
			annotations = Solid(
				tool = note_distance(wrist, tool, project=Z, offset=forearm*0.8*Y),
				),
			),
	})
	l = link_outer(
				actuators.elbow.deloc('shell', 'output'), 
				actuators.midfore.deloc('shell', 'output'),
				driver=True,
				)
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
			),
		'back_back': Solid(
			joint = actuators.midback,
			),
		'back_fore': Solid(
			joint = actuators.elbow,
			),
		'fore_back': Solid(
			joint = actuators.midfore,
			),
		'fore_fore': Solid(
			joint = actuators.wrist,
			),
		'wrist': Solid(
			joint = actuators.tool,
			),
	})
	return kinematic

if __name__ == '__madcad__':
	settings.resolution = ('sqradm', 1.)
#	settings.resolution = ('sqradm', 0.4)
	
	arm = arm_repeated(210, 210)
#	a = arm_alternate(200, 200)
	arm.default = [0, pi/3, pi/3, -pi/3, -pi/6, pi/4, 0]

#	export(arm, f"{__file__}/../out/arm-repeated-v0.2", (200, 200))
