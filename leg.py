from madcad import *
from madcad.joints import *
from madcad.assembly import *
from madcad.scheme import *

from foot import foot
from passive_joint import passive_joint
from gearbox.strainwave import strainwave
from pnprint import nprint



def circular_screwing(axis, radius, height, dscrew, diameters:int=1, div:int=8, hold:float=0) -> '(Mesh, list)':
	''' holes and bolts to screw a circular perimeter
	
		Parameters:
			axis:  placement of circle around which the screws are placed
			radius: radius of the perimeter on which to put the screws
			height: bolts length
			dscrew: diameter of the biggest screws
			diameters: each biggest screw is followed by this number-1 of additional smaller diameters
			div: number of biggest screws distributed uniformly on the perimeter
			hold: 
				if non zero, holding screws of the given length are added to help the assembly.
				if True, the holding screws length is automatically determined.
				
		Return: a tuple (holes:Mesh, bolts:list) where the bolts is a list of `Solid`
	'''
	holes = Mesh()
	bolts = []
	x,y,z = dirbase(axis.direction)
	a = axis.origin + radius*x
	b = axis.origin + radius*x + height*z
	gap = 0.1*dscrew*z
	enlarge = 1.05
	diameters_list = []
	for i in range(diameters):
		d = enlarge*stfloor(0.8**i * dscrew)
		diameters_list.append(d)
		holes += cylinder(a-gap, b+gap, d/2) .transform(rotatearound(i*1.4*dscrew/radius, axis))
	holes = repeataround(holes, div, axis).flip()
	screwing = Solid(
		interface = Solid(
			perimeter = Circle(axis, radius),
			div = div,
			dscrew = dscrew,
			diameters = diameters_list,
			height = height,
			annotations = Solid(
				perimeter = note_leading(
					axis.origin + radius*normalize(x+y*0.5), 
					offset=radius*0.2*(normalize(x+y*0.5)-z), 
					text='R {:g}'.format(radius)),
				screw = note_leading(
					axis.origin + radius*x + dscrew*0.5*x, 
					offset=radius*0.2*(x-z), 
					text='{}x M{:g}x{:g}'.format(div, dscrew, height)),
				),
			),
		holes = holes,
		placeholder = bolt(a, b, dscrew),
		)
	if hold is True:
		hold = dscrew
	if hold:
		angle = 1.7*dscrew/radius
		holes += repeataround(screw_slot(Axis(a+gap,-z), dscrew, 
					screw=height-hold, 
					hole=hold, 
					flat=True), 2) .transform(rotatearound(-angle, axis))
		screwing.hold = [
			screw(dscrew, height, head='flat')
				.place((Revolute, Axis(O,Z), Axis(a,-z)))
				.transform(rotatearound(-angle, axis)),
			screw(dscrew, height, head='flat')
				.place((Revolute, Axis(O,Z), Axis(a,-z)))
				.transform(rotatearound(pi-angle, axis)),
			note_leading(
				rotatearound(-angle, axis)*a, 
				offset=radius*0.2*(x-z), 
				text='2x M{:g}x{:g} HS'.format(dscrew, height-hold)),
			]
	return screwing

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



def references(
	leg_length,
	backleg_length,
	offset_shoulder,
	offset_traverse,
	offset_scapula,
	foot_clearance,
	gearbox_shoulder,
	gearbox_leg,
	):
	scapula = -offset_scapula*X
	shoulder = -offset_shoulder*Y
	backknee = -backleg_length*Z - offset_shoulder*Y
#	backknee = shoulder - leg_length/2*Z
	foreknee = backknee +offset_traverse*Y -(leg_length-backleg_length)/2*X
	foot_center = foreknee -(leg_length-backleg_length)/2*Z -foot_clearance*Z
	dscrew = 5
	gearbox_radius = gearbox_leg + dscrew
	passive_radius = 20 + dscrew
	knees = [
		Z*gearbox_radius,
		-Z*passive_radius +Z*dscrew +X*+passive_radius,
		-Z*passive_radius +Z*dscrew +X*-passive_radius,
		]
#	knees = regon(Axis(O,Y), knee_radius, 3, alignment=Z).points

	return Solid(**vars())


def joint_slot(interface):
	play = 0.1
	x,y,z = dirbase(-interface.perimeter.axis.direction)
	hole = revolution(wire([
		-interface.height*3*z + interface.perimeter.radius*x + interface.diameters[0]*1.5*x,
		+play*interface.perimeter.axis.direction + interface.perimeter.radius*x + interface.diameters[0]*1.5*x,
		+play*interface.perimeter.axis.direction + interface.perimeter.radius*x - interface.diameters[0]*x,
		interface.diameters[0]*z + interface.perimeter.radius*x - interface.diameters[0]*x,
		interface.diameters[0]*3*z + interface.perimeter.radius*x + interface.diameters[0]*0.5*x,
		interface.perimeter.radius*3*z + interface.perimeter.radius*x + interface.diameters[0]*0.5*x,
		]).segmented()).orient().flip().transform(interface.perimeter.center)
#	screwing = circular_screwing(interface.perimeter.axis, interface.perimeter.radius, interface.height, stfloor(interface.diameters[0], 0.3))
	screwing = Mesh()
	for p in regon(interface.perimeter.axis.offset(-interface.dscrew), interface.perimeter.radius, interface.div).points:
		screwing += screw_slot(
			Axis(p, -interface.perimeter.axis.direction), 
			dscrew = interface.dscrew, 
			rslot = 1.3*interface.dscrew, 
			hole = interface.height, 
			expand = 2*interface.diameters[0],
			)
	return intersection(hole, screwing)

def backleg_body(refs, shoulder, gearbox, passive1, passive2):
	left = convexoutline(web([
		Circle(Axis(refs.shoulder + shoulder.output.perimeter.radius*Z, X), shoulder.output.perimeter.radius*0.6),
		Circle(Axis(mix(refs.shoulder, refs.backknee, 0.3), X), gearbox.output.perimeter.radius),
		Circle(Axis(refs.backknee - gearbox.output.perimeter.radius*Z, X), gearbox.output.perimeter.radius*0.5),
		]))
	right = convexoutline(web([
#		web(gearbox.shell.output.perimeter).transform(gearbox.pose),
		Circle(shoulder.output.perimeter.axis.transform(shoulder.pose), shoulder.output.perimeter.radius + shoulder.output.diameters[0]*1.5),
		Circle(Axis(mix(
				shoulder.output.perimeter.axis.transform(shoulder.pose).origin, 
				gearbox.output.perimeter.axis.transform(gearbox.pose).origin, 
				0.5), Y), gearbox.output.perimeter.radius*1.3),
		Circle(gearbox.output.perimeter.axis.transform(gearbox.pose), gearbox.output.perimeter.radius + gearbox.output.diameters[0]*1.5),
		Circle(passive1.perimeter.axis.transform(passive1.pose), passive1.perimeter.radius + gearbox.output.diameters[0]*1.5),
		Circle(passive2.perimeter.axis.transform(passive2.pose), passive2.perimeter.radius + gearbox.output.diameters[0]*1.5),
		]))
	top = web(Circle(Axis(refs.shoulder, Z), gearbox.output.perimeter.radius*1.4))
	body = intersection(
		intersection(
			extrusion(flatsurface(right), -gearbox.output.perimeter.radius*2*Y).orient(),
			extrusion(left, gearbox.output.perimeter.radius*3*X, alignment=0.5).orient(),
			),
		extrusion(top, -refs.leg_length*Z, alignment=0.5).orient(),
		)
	body = intersection(body, saddle(
		flatsurface(Ellipsis(
			shoulder.output.perimeter.axis.transform(shoulder.pose).offset(-gearbox.output.perimeter.radius*0.6).origin,
			gearbox.output.perimeter.radius*0.8*X,
			gearbox.output.perimeter.radius*0.4*Y,
			)),
		Segment(
			shoulder.output.perimeter.axis.transform(shoulder.pose).origin,
			gearbox.output.perimeter.axis.transform(gearbox.pose).origin,
			),
		).orient().flip())
	body1 = intersection(
		body, 
		joint_slot(shoulder.output)
			.transform(rotatearound(pi/8, shoulder.output.perimeter.axis))
			.transform(shoulder.pose)
		)
	body2 = intersection(body1, joint_slot(passive1).transform(passive1.pose))
	body3 = intersection(body2, joint_slot(passive2).transform(passive2.pose))
	body4 = intersection(
		body3,
		joint_slot(gearbox.output)
			.transform(rotatearound(pi/8, gearbox.output.perimeter.axis))
			.transform(gearbox.pose),
		)
	
	return Solid(
		body = body4.finish(),
		shoulder = shoulder,
		gearbox = gearbox,
		passive1 = passive1,
		passive2 = passive2,
		)

def foreleg_body(refs, leading, edge1, edge2, foot):
	w = 10
	left = Softened([
		refs.foreknee + leading.perimeter.radius*2.5*Z + leading.perimeter.radius*0.4*Y,
		refs.foreknee + leading.perimeter.radius*2.5*Z + leading.perimeter.radius*0.5*Y,
		refs.foreknee + leading.perimeter.radius*1*Y,
		mix(refs.foot_center, refs.foreknee, 0.45) + w*Y,
		mix(refs.foot_center, refs.foreknee, 0.4) + w*Y,
		])
#	right = convexoutline(web([
#		Circle(Axis(mix(refs.foot_center, refs.foreknee, 0.45) + w*Y, Y), 20),
#		Circle(Axis(mix(refs.foot_center, refs.foreknee, 0.45) - w*Y, Y), 20),
#		Circle(leading.perimeter.axis.transform(leading.pose), leading.perimeter.radius + leading.dscrew*1.5),
#		Circle(edge1.perimeter.axis.transform(edge1.pose), edge1.perimeter.radius + edge1.dscrew*1.5),
#		Circle(edge2.perimeter.axis.transform(edge2.pose), edge2.perimeter.radius + edge2.dscrew*1.5),
#		]))
#	body = intersection(
#			extrusion(flatsurface(right), leading.perimeter.radius*2*Y).orient(),
#			extrusion(left, leading.perimeter.radius*3*X, alignment=0.5).flip(),
#			)
	right = convexhull(mesh.mesh([
		cylinder(
			mix(refs.foot_center, refs.foreknee, 0.45) + w*Y,
			mix(refs.foot_center, refs.foreknee, 0.45) - w*Y, 
			20),
		cylinder(
			leading.perimeter.axis.transform(leading.pose).origin, 
			leading.perimeter.axis.transform(leading.pose).offset(-leading.perimeter.radius*0.5).origin, 
			leading.perimeter.radius + leading.dscrew*1.5),
		cylinder(
			edge1.perimeter.axis.transform(edge1.pose).origin, 
			edge1.perimeter.axis.transform(edge1.pose).offset(-edge1.perimeter.radius*0.5).origin, 
			edge1.perimeter.radius + edge1.dscrew*1.5),
		cylinder(
			edge2.perimeter.axis.transform(edge2.pose).origin, 
			edge2.perimeter.axis.transform(edge2.pose).offset(-edge2.perimeter.radius*0.5).origin, 
			edge2.perimeter.radius + edge2.dscrew*1.5),
		])).mergegroups()
	body = intersection(right, 
		extrusion(left, leading.perimeter.radius*3*X, alignment=0.5).flip(),
		)
		
	r = leading.perimeter.radius
	body1 = intersection(
			body,
			joint_slot(leading).transform(leading.pose)
			+ extrusion(
				Circle(Axis(mix(refs.foot_center, refs.foreknee, 0.55), Y), r*0.3),
				r*3*Y, alignment=0.5,
				).flip()
			+ extrusion(
				Circle(Axis(mix(refs.foot_center, refs.foreknee, 0.72), Y), r*0.4),
				r*3*Y, alignment=0.5,
				).flip()
			)
	body2 = intersection(body1, joint_slot(edge1).transform(edge1.pose))
	body3 = intersection(body2, joint_slot(edge2).transform(edge2.pose))

	return Solid(
		body = body3.finish(),
		leading = leading,
		edge1 = edge1,
		edge2 = edge2,
		foot = foot,
		)

bone_curving = -1.2
color_traverse = vec3(0.8, 0.4, 0.1)*0.5

def traverse_bone(start, stop, screw_height, rext):
	return extrans(
#		flatsurface(wire(Circle(Axis(O, Y), rext))).flip(),
		flatsurface(wire(Ellipsis(O, Z*rext, X*rext + screw_height*0.5*Y))).flip(),
		[translate(start) * mat4(scaledir(Y, 0))]
		+ [translate(mix(
			start + screw_height*Y,
			stop - screw_height*Y,
			x)) * scale(vec3(1 + x*(1-x) * bone_curving))
			for x in linrange(0, 1, div=20)]
		+ [translate(stop) * mat4(scaledir(Y, 0))],
		)

def traverse_main(start, stop, gearbox):
	play = 0.1
	screw_height = 5
	rext = gearbox.output.perimeter.radius + 1.5*gearbox.output.diameters[0]
	rint = gearbox.output.perimeter.radius - 2*gearbox.output.diameters[0]
	bone = traverse_bone(start, stop, screw_height, rext)
	passthrough = (
		cylinder(start - play*Y, start + project(stop-start, Y), rint).flip()
		+ cylinder(stop + play*Y, stop + project(start-stop, Y), rint).flip()
		)
	screwing = Mesh()
	dscrew = floor(gearbox.output.diameters[0])
	for p in regon(Axis(start, Y).offset(screw_height), gearbox.output.perimeter.radius, 8).points:
		screwing += screw_slot(
			Axis(p,Y), 
			dscrew=dscrew, 
			rslot=1.3*dscrew, 
			hole=2*screw_height, 
			expand=dot(stop-start, Y),
			)
	for p in regon(Axis(stop, -Y).offset(screw_height), gearbox.output.perimeter.radius, 8).points:
		screwing += screw_slot(
			Axis(p,-Y), 
			dscrew = dscrew, 
			rslot = 1.3*dscrew, 
			hole = 2*screw_height, 
			expand = dot(stop-start, Y),
			)
	holes = Mesh()
	rhole = distance(start, stop) * 0.06
	l = distance(start, stop)
	rx = 1.8 * gearbox.output.perimeter.radius / l
	for x in linrange(1-rx, rx, step=2.5*rhole/l):
		p = mix(start, stop, x)
		holes += cylinder(p - 2*rhole*Y, p + 2*rhole*Y, rhole).flip()
#	screwing, bolts = circular_screwing(
#		Axis(start, Y).offset(screw_height), 
#		gearbox.output.perimeter.radius, 
#		2*screw_height, 
#		floor(gearbox.output.diameters[0]),
#		)
	return intersection(bone, passthrough + screwing + holes) .option(color=color_traverse)

def traverse_secondary(start, stop):
	screw_height = 5
	r = 15
	bone = traverse_bone(start, stop, screw_height, r)
	return bone .option(color=color_traverse)


def assemble(refs):
	joints = [
		Revolute(('chest', 'shoulder'), Axis(refs.scapula, X)),
		Revolute(('shoulder', 'backleg'), Axis(refs.shoulder, Y)),
#		PointSlider(('chest', 'foreleg'), Axis(refs.foot_center, Z)),
#		Ball(('foreleg', 'some'), refs.foot_center),
#		Planar(('some', 'chest'), Axis(refs.foot_center, Z)),
		]
	for i, p in enumerate(refs.knees):
		joints.extend([
			Revolute(('backleg', f'edge-{i}'), Axis(refs.backknee + p, Y)),
			Revolute(('foreleg', f'edge-{i}'), Axis(refs.foreknee + p, Y)),
			])

	
	# for scapula joint
	gearbox = strainwave(refs.gearbox_shoulder, 60, nscrews=8)
	passive = passive_joint(40, dscrew=5, nscrew=8)
	scapula = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.scapula, -X))),
		passive = passive.place((Revolute, passive.inner.perimeter.axis, Axis(-refs.scapula, X))),
		)

	gearbox = strainwave(refs.gearbox_shoulder, 60, nscrews=8)
	shoulder = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.shoulder, -Y))),
		)

	# for backnee joint
	gearbox = strainwave(refs.gearbox_leg, 60, nscrews=8)
	passive = passive_joint(20, dscrew=4, nscrew=4)
	backknee = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.backknee + refs.knees[0], -Y))),
		edge1 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[1], -Y)))
			* rotate(0.5, Z)
			),
		edge2 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[2], -Y)))
			* rotate(1.1, Z)
			),
		)

	# for foreknee joint
	passive_big = passive_joint(50, dscrew=5, nscrew=8)
	foreknee = Solid(
		edge0 = passive_big.place((Revolute, passive_big.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[0], Y))),
		edge1 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[1], Y)))
			* rotate(1.1, Z)
			),
		edge2 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[2], Y)))
			* rotate(0.5, Z)
			),
		)

	# subsystem managing contact with the ground
	size = refs.gearbox_leg*2
	foot1 = foot(
		front_size = stceil(size), 
		side_size = stceil(size*0.9),
		parallelogram_height = stceil(size*0.4),
		).transform(placement((Revolute, Axis(O,Z), Axis(refs.foot_center, Z))) * rotate(pi, Z))

	backleg = backleg_body(refs, 
		shoulder.gearbox.deloc('shell'),
		backknee.gearbox.deloc('shell'), 
		backknee.edge1.deloc('outer'), 
		backknee.edge2.deloc('outer'),
		)
	foreleg = foreleg_body(refs,
		foreknee.edge0.deloc('outer'),
		foreknee.edge1.deloc('outer'),
		foreknee.edge2.deloc('outer'),
		foot1,
		)

	return Kinematic(joints, ground='chest', content={
		'chest': Solid(
			gearbox = scapula.gearbox.deloc('shell'),
			passive = scapula.passive.deloc('outer'),
			),
		'shoulder': Solid(
			shoulder = shoulder.gearbox.deloc('output'),
			scapula = scapula.gearbox.deloc('output'),
			passive = scapula.passive.deloc('inner'),
			),
		'backleg': backleg,
		'foreleg': foreleg,
		'edge-0': Solid(
			body = traverse_main(refs.backknee + refs.knees[0], refs.foreknee + refs.knees[0], backknee.gearbox),
			start = backknee.gearbox.deloc('output'),
			stop = foreknee.edge0.deloc('inner'),
			),
		'edge-1': Solid(
			body = traverse_secondary(refs.backknee + refs.knees[1], refs.foreknee + refs.knees[1]),
			start = backknee.edge1.deloc('inner'),
			stop = foreknee.edge1.deloc('inner'),
			),
		'edge-2': Solid(
			body = traverse_secondary(refs.backknee + refs.knees[2], refs.foreknee + refs.knees[2]),
			start = backknee.edge2.deloc('inner'),
			stop = foreknee.edge2.deloc('inner'),
			),
		})


settings.resolution = ('sqradm', 1.)

refs = references(
	leg_length = 550,
	backleg_length = 165,
	offset_scapula = 20,
	offset_shoulder = 58,
	offset_traverse = 60,
	foot_clearance = 60,
	gearbox_shoulder = 50,
	gearbox_leg = 50,
	)
assembly = assemble(refs)


# notes:
# - shoulder actuator should be stronger than backleg on front legs 
# - shoulder actuator should be weaker than backleg on rear legs
# - small traverses should not have bearings to redice bulkiness