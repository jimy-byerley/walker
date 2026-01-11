from madcad import *
from madcad.joints import *
from madcad.assembly import *

from foot import foot
from passive_joint import passive_joint
from gearbox.strainwave import strainwave
from pnprint import nprint


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

	gearbox = strainwave(refs.gearbox_shoulder, 60, nscrews=8)
	passive = passive_joint(40, dscrew=5, nscrew=8)
	
	chest = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.scapula, -X))),
		passive = passive.place((Revolute, passive.inner.perimeter.axis, Axis(-refs.scapula, X))),
		)

	gearbox = strainwave(refs.gearbox_leg, 60, nscrews=8)
	passive = passive_joint(20, dscrew=4, nscrew=4)
	backleg = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.backknee + refs.knees[0], -Y))),
		passive1 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[1], -Y)))
			* rotate(1.1, Z)
			),
		passive2 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[2], -Y)))
			* rotate(0.5, Z)
			),
		)

	backleg.body = backleg_body(refs, gearbox, passive)
	
	passive_big = passive_joint(50, dscrew=5, nscrew=8)

	size = refs.gearbox_leg*2
	foot1 = foot(
		front_size = stceil(size), 
		side_size = stceil(size*0.9),
		parallelogram_height = stceil(size*0.4),
		)
	foreleg = Solid(
		passive0 = passive_big.place((Revolute, passive_big.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[0], Y))),
		passive1 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[1], Y)))
			* rotate(0.5, Z)
			),
		passive2 = passive.transform(
			placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[2], Y)))
			* rotate(1.1, Z)
			),
		body = convexoutline(web([
			Circle(Axis(mix(refs.foot_center, refs.foreknee, 0.55), Y), 20),
			Circle(Axis(refs.foreknee + refs.knees[0], Y), gearbox.shell.output.perimeter.radius + gearbox.shell.output.diameters[0]),
			Circle(Axis(refs.foreknee + refs.knees[1], Y), passive.outer.perimeter.radius + gearbox.shell.output.diameters[0]),
			Circle(Axis(refs.foreknee + refs.knees[2], Y), passive.outer.perimeter.radius + gearbox.shell.output.diameters[0]),
			])),
		foot = foot1.transform(placement((Revolute, Axis(O,Z), Axis(refs.foot_center, Z))) * rotate(pi, Z)),
		)
	
	gearbox = strainwave(refs.gearbox_shoulder, 60, nscrews=8)
	shoulder = Solid(
		gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.shoulder, -Y))),
		)

	edge0 = traverse_main(refs.backknee + refs.knees[0], refs.foreknee + refs.knees[0], gearbox)
	edge1 = traverse_secondary(refs.backknee + refs.knees[1], refs.foreknee + refs.knees[1])
	edge2 = traverse_secondary(refs.backknee + refs.knees[2], refs.foreknee + refs.knees[2])
	

#	l = foreleg(refs, passive_big, passive)
#	content['foreleg'].some = foreleg(refs, content['foreleg'].passive0, content['foreleg'].passive1)
	


	return Kinematic(joints, ground='chest', content={
		'chest': chest,
		'shoulder': shoulder,
		'backleg': backleg,
		'foreleg': foreleg,
		'edge-0': edge0,
		'edge-1': edge1,
		'edge-2': edge2,
		})

def backleg_body(refs, gearbox, passive):
	left = convexoutline(web([
		Circle(Axis(refs.shoulder + gearbox.shell.output.perimeter.radius*Z, X), gearbox.shell.output.perimeter.radius*0.6),
		Circle(Axis(mix(refs.shoulder, refs.backknee, 0.3), X), gearbox.shell.output.perimeter.radius),
		Circle(Axis(refs.backknee - gearbox.shell.output.perimeter.radius*Z, X), gearbox.shell.output.perimeter.radius*0.5),
		]))
	right = convexoutline(web([
#		web(gearbox.shell.output.perimeter).transform(gearbox.pose),
		Circle(Axis(refs.shoulder, Y), gearbox.shell.output.perimeter.radius + gearbox.shell.output.diameters[0]),
		Circle(Axis(mix(refs.shoulder, refs.backknee, 0.4), Y), gearbox.shell.output.perimeter.radius*1.3),
		Circle(Axis(refs.backknee + refs.knees[0], Y), gearbox.shell.output.perimeter.radius + gearbox.shell.output.diameters[0]),
		Circle(Axis(refs.backknee + refs.knees[1], Y), passive.outer.perimeter.radius + gearbox.shell.output.diameters[0]),
		Circle(Axis(refs.backknee + refs.knees[2], Y), passive.outer.perimeter.radius + gearbox.shell.output.diameters[0]),
		])).transform(-20*Y)
	top = web(Circle(Axis(refs.shoulder, Z), gearbox.shell.output.perimeter.radius*1.4))
	body = intersection(
		intersection(
			extrusion(flatsurface(right), -gearbox.shell.output.perimeter.radius*2*Y).orient(),
			extrusion(left, gearbox.shell.output.perimeter.radius*3*X, alignment=0.5).orient(),
			),
		extrusion(top, -refs.leg_length*Z, alignment=0.5).orient(),
		)
#	body = intersection(
#		body, 
#		joint_slot(gearbox.shell.output).transform(placement((Revolute, Axis(O,Z), Axis(refs.shoulder, Y).offset(20)))),
#		)
	
	return body

def joint_slot(interface):
	return revolution(web([
#		interface.
		]))

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

	


settings.resolution = ('sqradm', 1.)

#refs = references(
#	leg_length = 400,
#	traverse_length = 200,
#	offset_shoulder = 40,
#	offset_traverse = 90,
#	foot_clearance = 50,
#	knee_radius = 50,
#	)
refs = references(
	leg_length = 550,
	backleg_length = 160,
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