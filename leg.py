from madcad import *
from madcad.joints import *
from madcad.assembly import *

from foot import foot
from passive_joint import passive_joint
from gearbox.strainwave import strainwave
from pnprint import nprint

settings.resolution = ('sqradm', 0.8)

def references(
	leg_length,
	traverse_length,
	offset_shoulder,
	offset_traverse,
	foot_clearance,
	knee_radius,
	):
	shoulder = +offset_shoulder*Y
	backknee = -leg_length/2*Z +offset_shoulder*Y + 0.2*offset_traverse*Y
	foreknee = backknee +offset_traverse*Y -traverse_length*X
	foot_center = foreknee -leg_length/2*Z -foot_clearance*Z
	knees = regon(Axis(O,Y), knee_radius, 3, alignment=Z).points

	return Solid(**vars())

def assemble(refs):
	joints = [
		Revolute(('chest', 'shoulder'), Axis(-refs.offset_shoulder*X, X)),
		Revolute(('shoulder', 'backleg'), Axis(refs.shoulder, Y)),
		Ball(('foreleg', 'foot'), refs.foot_center),
		]
	for i, p in enumerate(refs.knees):
		joints.extend([
			Revolute(('backleg', f'edge-{i}'), Axis(refs.backknee + p, Y)),
			Revolute(('foreleg', f'edge-{i}'), Axis(refs.foreknee + p, Y)),
			])
	content={
		'chest': strainwave(50, 60).place((Revolute, Axis(O,Z), Axis(-2*refs.offset_shoulder*X, -X))),
		'shoulder': Solid(),
		'backleg': Solid(),
		'foreleg': Solid(),
		'edge-0': Solid(),
		'edge-1': Solid(),
		'edge-2': Solid(),
		}

	gearbox = strainwave(50, 60)
	content['backleg'].gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.backknee + refs.knee_radius*Z, -Y)))
#	content['backleg'].passive1 = passive_joint(50, dscrew=5, nscrew=8)
#	content['backleg'].body = backleg()
	passive = passive_joint(30, dscrew=5, nscrew=4)
	content['backleg'].passive1 = passive.transform(
		placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[1], -Y)))
		* rotate(1.3, Z)
		)
	content['backleg'].passive2 = passive.transform(
		placement((Revolute, passive.inner.perimeter.axis, Axis(refs.backknee + refs.knees[2], -Y)))
		* rotate(0.3, Z)
		)
	content['foreleg'].passive1 = passive.transform(
		placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[1], Y)))
		* rotate(0.3, Z)
		)
	content['foreleg'].passive2 = passive.transform(
		placement((Revolute, passive.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[2], Y)))
		* rotate(1.3, Z)
		)
	passive_big = passive_joint(50, dscrew=5, nscrew=8)
	content['foreleg'].passive0 = passive_big.place((Revolute, passive_big.inner.perimeter.axis, Axis(refs.foreknee + refs.knees[0], Y)))
	
	gearbox = strainwave(50, 60)
	content['shoulder'].gearbox = gearbox.place((Revolute, gearbox.output.perimeter.axis, Axis(refs.shoulder, Y)))
	content['edge-0'].body = traverse_main(refs.backknee + refs.knees[0], refs.foreknee + refs.knees[0], gearbox)
	content['edge-1'] = traverse_secondary(refs.backknee + refs.knees[1], refs.foreknee + refs.knees[1])
	content['edge-2'] = traverse_secondary(refs.backknee + refs.knees[2], refs.foreknee + refs.knees[2])
	
#	l = foreleg(refs, passive_big, passive)
#	content['foreleg'].some = foreleg(refs, content['foreleg'].passive0, content['foreleg'].passive1)

	foot1 = foot(front_size=50*2, side_size=45*2, parallelogram_height=20*2)
	content['foreleg'].foot = foot1.transform(placement((Revolute, Axis(O,Z), Axis(refs.foot_center, Z))) * rotate(pi, Z))
	
	return Kinematic(joints, ground='chest', content=content)

def backleg(refs):
	o1 = convexhull(web([
		Circle(Axis(refs.shoulder,Y), gearbox.rext + gearbox.shell.output.diameters[0]),
		Circle(Axis(refs.shoulder,Y), gearbox.rext + gearbox.shell.output.diameters[0]),
		Circle(Axis(mix(refs.backknee, refs.shoulder, 0.5), Z), refs.knee_radius*0.7),
		]))
	o2 = convexhull(web([
		wire(content['backleg'].gearbox.shell.output.perimeter).transform(content['backleg'].gearbox.pose), 
		Circle(Axis(backknee + knees[1],Y), knee_radius*0.5),
		Circle(Axis(backknee + knees[2],Y), knee_radius*0.5),
		]))

bone_curving = -1.2
color_traverse = vec3(0.8, 0.4, 0.1)*0.5

def traverse_main(start, stop, gearbox):
	play = 0.1
	screw_height = 5
	rext = gearbox.output.perimeter.radius + 1.5*gearbox.output.diameters[0]
	rint = gearbox.output.perimeter.radius - 2*gearbox.output.diameters[0]
	bone = extrans(
		flatsurface(wire(Circle(Axis(O, Y), rext))).flip(),
		[translate(start)]
		+ [translate(mix(
			start + screw_height*Y,
			stop - screw_height*Y,
			x)) * scale(vec3(1 + x*(1-x) * bone_curving))
			for x in linrange(0, 1, div=20)]
		+ [translate(stop)],
		)
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
	for x in linrange(0.3, 0.7, div=1):
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
	r = 20
	e = extrans(
		Circle(Axis(O, Y), r),
		[translate(start)]
		+ [translate(mix(
			start + screw_height*Y,
			stop - screw_height*Y,
			x)) * scale(vec3(1 + x*(1-x) * bone_curving))
			for x in linrange(0, 1, div=20)]
		+ [translate(stop)],
		)
#	f = intersection(e, mesh.mesh([cylinder(mix(refs.backknee
	return e .option(color=color_traverse)

def foreleg(refs, main, secondary):
	side = refs.foreknee +main.inner.height*Y +main.outer.height*Y
	hull = convexhull(mesh.mesh([
		icosphere(O, main.outer.perimeter.radius + main.outer.diameters[0]) 
			.transform(translate(side + refs.knees[0]) * mat4(scaledir(Y, 0.3))),
		icosphere(O, secondary.outer.perimeter.radius + secondary.outer.diameters[0])
			.transform(translate(side + refs.knees[1]) * mat4(scaledir(Y, 0.4))),
		icosphere(O, secondary.outer.perimeter.radius + secondary.outer.diameters[0])
			.transform(translate(side + refs.knees[2]) * mat4(scaledir(Y, 0.4))),
		icosphere(O, 10)
			.transform(translate(mix(refs.foot_center, refs.foreknee, 0.8) +main.inner.height*Y +main.outer.height*Y) * scale(vec3(4, 2, 10))),
		]))
	screwing = Mesh()
	dscrew = floor(main.outer.diameters[0])
	screw_height = dscrew
	for p in regon(Axis(side + refs.knees[0], Y).offset(screw_height), main.outer.perimeter.radius, 8).points:
		screwing += screw_slot(
			Axis(p,Y), 
			dscrew=dscrew, 
			rslot=1.3*dscrew, 
			hole=2*screw_height, 
			expand = main.outer.perimeter.radius,
			)
	return intersection(hull, screwing)

def foreleg(refs, main, secondary):
	side = refs.foreknee +main.inner.height*Y +main.outer.height*Y
	a = convexoutline([
		Circle(Axis(side + refs.knees[0], Y), main.outer.perimeter.radius + main.outer.diameters[0]),
		Circle(Axis(side + refs.knees[1], Y), main.outer.perimeter.radius + main.outer.diameters[1]),
		Circle(Axis(side + refs.knees[2], Y), main.outer.perimeter.radius + main.outer.diameters[2]),
		])
	b = convexoutline
	
refs = references(
	leg_length = 400,
	traverse_length = 200,
	offset_shoulder = 40,
	offset_traverse = 100,
	foot_clearance = 50,
	knee_radius = 50,
	)
assembly = assemble(refs)
