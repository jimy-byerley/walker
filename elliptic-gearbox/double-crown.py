from madcad import *
from madcad.gear import gearprofile, rackprofile
from madcad.joints import *
import numpy as np
import scipy.optimize
from functools import reduce, partial
import operator



def perimeter(f, start, stop, div=128):
	points = [f(x)  for x in linrange(start, stop, div=div)]
	return sum(distance(points[i-1], points[i])  for i in range(1, len(points)))
	
def flex_args(flex, circle_radius, ratio):
	flex_max = circle_radius
	circle_perimeter = 2*pi*circle_radius
	def predicate(flex_min):
		flex_perimeter = perimeter(lambda phase: flex(flex_max, flex_min, phase), 0, 2*pi)
		return flex_perimeter > circle_perimeter*ratio
	flex_min = fbisect(predicate, circle_radius, 0)
	return flex_max, flex_min

def homogeneous_spacing(flex, start, stop, divs):
	def cost(phases):
		points = np.array([
			distance(
				flex(phases[i-1]), 
				flex(phases[i]))
			for i in range(len(phases))
			])
		return points - np.mean(points)
	phases = np.linspace(start, stop, divs, endpoint=False)
	result = scipy.optimize.least_squares(cost, phases)
	assert result.success
	phases = result.x
	phases -= phases[0]
	return phases

def double_crown_meshing(circle_radius, circle_teeth):
	# ellipsis
	# def flex(a, b, phase):
		# return vec2(a*cos(phase), b*sin(phase))
	# def dflex(a, b, phase):
	# 	return vec2(-a*sin(phase), b*cos(phase))
	
	# offseted ellipsis
	# def flex1(a, b, phase):
	# 	return vec2(a*cos(phase), b*sin(phase))
	# def dflex1(a, b, phase):
	# 	return vec2(-a*sin(phase), b*cos(phase))
	# e = maxradius*0.5
	# def flex(a, b, phase):
	# 	return flex1(a-e, b-e, phase) - perp(normalize(dflex1(a-e, b-e, phase))) * e
	# def dflex(a, b, phase):
	# 	dx = 1e-6
	# 	return (flex(a,b,phase+dx) - flex(a,b,phase-dx)) / (2*dx)
	
	# sine deformation wave
	def flex(a, b, phase):
		return vec2(cos(phase), sin(phase)) * mix(b, a, linstep(-1, 1, cos(2*phase)))
	def dflex(a, b, phase):
		dx = 1e-6
		return (flex(a,b,phase+dx) - flex(a,b,phase-dx)) / (2*dx)

	# ramps
	#def flex(a, b, phase):
	#	quater = phase % pi
	#	if quater > pi/2:	quater = pi-quater
	#	return vec2(cos(phase), sin(phase)) * mix(a, b, quater*2/pi)
	#def dflex(a, b, phase):
	#	dx = 1e-6
	#	return (flex(a, b, phase) - flex(a, b, phase-dx))/dx

	flex_teeth = circle_teeth-2
	flex_mean = flex_teeth/circle_teeth * circle_radius
	flex_max, flex_min = flex_args(flex, circle_radius, ratio=flex_teeth/circle_teeth)

	phases = homogeneous_spacing(partial(flex, flex_max, flex_min), 0, 2*pi, flex_teeth)
	print('flex', flex_min, flex_max)
	
	base = wire(vec3(flex(flex_max, flex_min, phase),0)  for phase in phases).close()

	teeth = Solid()
	teeth.step = 2*pi*circle_radius / (circle_teeth-3)
	teeth.height = 0.3*teeth.step
	teeth.play = teeth.height*0.2
	tooth = (
		gearprofile(teeth.step, circle_teeth, 
			height=teeth.height, 
			asymetry=-teeth.height*0.2, 
			offset=-teeth.play,
			pressure_angle=radians(30),
			)
		.transform(-circle_radius*X)
		)
	tooth.indices.pop()
	tooth.tracks.pop()

	teeth.flex_deformed = wire([tooth.transform(
		translate(vec3(flex(flex_max, flex_min, phase), 0))
		* mat4(quat(Y, vec3(normalize(dflex(flex_max, flex_min, phase)), 0)))
		) for phase in phases]).close().option(color=vec3(1,1,0))
	teeth.flex_initial = repeat(
		tooth.transform(+circle_radius * flex_teeth/circle_teeth * X),
		flex_teeth,
		rotate(2*pi/flex_teeth, Z),
		).close().option(color=vec3(1,0.2,0))

	teeth.crown_in = repeat(
		gearprofile(teeth.step, circle_teeth, 
			height=teeth.height, 
			asymetry=teeth.height*0.2, 
			pressure_angle=radians(30),
			),
		circle_teeth, 
		rotate(2*pi/circle_teeth, Z),
		).close().option(color=vec3(0,0.5,1))
	teeth.crown_out = repeat(
		gearprofile(teeth.step, circle_teeth, 
			height=teeth.height, 
	#		asymetry=teeth_height*0.2, 
			offset=teeth.height*0.2,
			pressure_angle=radians(33),
			),
		flex_teeth, 
		rotate(2*pi/flex_teeth, Z),
		).close().option(color=vec3(0,1,0.5))
	circle = Circle(Axis(O,Z), circle_radius)
		
	return Solid(**vars())


def elliptic_circulating(rball, rballs, nballs=None):
	if nballs is None:
		nballs = int(floor(2*pi*rballs / (3.3*rball)))
	
	balls_perimeter = Circle(Axis(O,Z), rballs)
	balls = repeataround(icosphere(rballs*X, rball), nballs).option(color=settings.colors['circulating'])
	
	cage_profile1 = Wire([
				vec3(cos(t), sin(t), 0) 
				* ( rballs + 0.55*rball * mix(1, -cos(t*nballs), 0.35) )    
				for t in linrange(0, 2*pi/nballs, div=20)])
	cage_profile2 = Wire([
				vec3(cos(t), sin(t), 0) 
				* ( rballs - 0.55*rball * mix(1, -cos(t*nballs), 0.35) )    
				for t in linrange(0, 2*pi/nballs, div=20)]).flip()
	cage_pattern = extrusion(
		flatsurface(wire([cage_profile1, cage_profile2]).close()).flip().transform(-0.8*rball*Z), 
		3*rball*Z)
	cage_pattern.mergeclose()
	
	spacing = rotate(2*pi/nballs, Z)
	slot = union(inflate(balls, rball*0.1), cone(rball*Z, -1.2*rball*Z, 1.5*rball).transform(rballs*X))
	cage_pattern = difference(cage_pattern.transform(vec3(0.001)), slot + slot.transform(spacing))
	cage = repeat(cage_pattern, nballs, spacing).option(color=settings.colors['bearing_cage'])
	return Solid(cage=cage, balls=balls)

def balls_guide_profile(rballs, rball, start, stop):
	return wire([vec3(rballs + rball*cos(t), 0, 1.05*rball*sin(t))
		for t in linrange(start, stop, div=10)])


def grooves_profile(radius, repetitions:int=16, alignment=0.5, angle=radians(40)) -> Wire:
	''' Coupling grooves profile 
		
		- The size of grooves depend on the pressure angle and the number of grooves
		
		Parameters:
		
			radius: the radius around which the grooves are placed
			repetitions: the number of grooves to put on the circle perimeter
			alignemnt: fraction of the grooves height
				- exterior grooves usually have alignment=1
				- interior grooves usually have alignemnt=0
			angle: pressure angle of the grooves
	'''
	h = radius/repetitions / tan(angle)
	def profile(t):
		return (radius+(0.5-alignment)*h+h*sin(repetitions*t)) * vec3(cos(t), sin(t), 0)
	c = Circle(Axis(O,Z), radius)
	return wire([ profile(t)
		for t in linrange(0, 2*pi, div=12*repetitions, end=False) ]).close()

def grooves(radius, height, repetitions:int=16, alignment=0.5, angle=radians(40)) -> Mesh:
	''' Coupling grooves surface
		
		- The bottom and top bevels direction depend on the alignment
		- The size of grooves depend on the pressure angle and the number of grooves
		
		Parameters:
		
			radius: the radius around which the grooves are placed
			height: the length of the grooves, including transition bevels
			repetitions: the number of grooves to put on the circle perimeter
			alignemnt: fraction of the grooves height
				- exterior grooves usually have alignment=1
				- interior grooves usually have alignemnt=0
			angle: pressure angle of the grooves
	'''
	h = radius/repetitions / tan(angle)
	if alignment > 0.5:    s = 1+h/radius
	else:                  s = 1-h/radius
	return extrans(grooves_profile(radius, repetitions, alignment, angle), [
				scale(vec3(0.1)),
				scale(vec3(s)),
				translate(h*Z),
				translate((height-h)*Z),
				translate(height*Z) * scale(vec3(s)),
				translate(height*Z) * scale(vec3(0.1)),
				])

def elliptic_double_crown(rext, nteeth, height=None, gap = 0.2, thickness = 0.9, rball = 3):
	if height is None:
		height = stceil(0.2*rext)

	interface = Solid(dscrew = 4)
	interface.perimeter = Circle(Axis(interface.dscrew*Z,Z), rext)
	interface.screw = Circle(interface.perimeter.axis.transform(interface.perimeter.radius*X), interface.dscrew/2)
	
	holes, bolts = circular_screwing(
		interface.perimeter.axis, 
		interface.perimeter.radius, 
		height, 
		interface.dscrew, 
		diameters=3)
	
	meshing = double_crown_meshing(
		circle_radius = stfloor(interface.perimeter.radius-interface.dscrew)*0.85, 
		circle_teeth = nteeth)
	
	crown_body = revolution(wire([
		(meshing.circle.radius)*X + meshing.teeth.height*Z + gap*Z,
		(meshing.circle.radius + 3*meshing.teeth.height)*X + gap*Z,
		(meshing.circle.radius + 4*meshing.teeth.height)*X + gap*Z,
		interface.screw.center - interface.dscrew*X,
		interface.screw.center + interface.dscrew*X,
		interface.screw.center + interface.dscrew*X + (height-interface.dscrew)*Z,
		(meshing.circle.radius + 3*meshing.teeth.height)*X + height*Z,
		(meshing.circle.radius)*X - meshing.teeth.height*Z + height*Z,
		]).segmented().flip())
	crown_body = intersection(crown_body, holes)
	crown_out = intersection(
		extrusion(meshing.teeth.crown_out, height*Z).flip(),
		crown_body,
		)
	crown_in = intersection(
		extrusion(meshing.teeth.crown_in, -height*Z),
		crown_body.transform(scaledir(Z,-1)).flip(),
		)
	
	rballs = rballs = meshing.flex_mean - rball - thickness
	
	flex_body = revolution(wire([
		meshing.flex_mean*X + height*Z + 3*meshing.teeth.height*X - meshing.teeth.height*Z,
		meshing.flex_mean*X + height*Z,
		meshing.flex_mean*X + height*Z - thickness*X,
		balls_guide_profile(rballs, rball, radians(40), -radians(40)).transform(0.5*height*Z),
		meshing.flex_mean*X - thickness*X + 0.1*height*Z,
		meshing.flex_mean*X - thickness*X,
		])).flip()
	flex_body = flex_body + flex_body.transform(scaledir(Z,-1)).flip()
	flex = intersection(flex_body, extrusion(meshing.teeth.flex_initial, 2*height*Z, alignment=0.5)).option(color=settings.colors['gear'])
	
	hole = stfloor((meshing.flex_min*0.8 - rball))
	generator_body = wire([
		(rballs*0.5)*X + height*Z,
		(rballs-rball*1.1)*X + height*Z,
		balls_guide_profile(rballs, rball, radians(-40)+pi, radians(90)+pi).transform(0.5*height*Z),
		(rballs+0.2*rball)*X + (0.5*height-1.1*rball)*Z,
		(rballs+0.2*rball)*X,
		])
	
	generator_body = extrans(generator_body.transform(-meshing.flex_mean*X), (
		translate(vec3(meshing.flex(meshing.flex_max, meshing.flex_min, t),0)) 
		* mat4(normalize(quat(Y, vec3(meshing.dflex(meshing.flex_max, meshing.flex_min, t), 0))))
		for t in linrange(0, 2*pi, div=200)
		))
	generator_body += generator_body.transform(scaledir(Z,-1)).flip()
	generator_body.mergeclose()
	hole_profile = wire([
		1.5*height*Z + 1.3*hole*X,
		0.9*height*Z + hole*X,
		-0.9*height*Z + hole*X,
		-1.5*height*Z + 1.3*hole*X,
		]).segmented().flip()
	generator_body = intersection(generator_body, revolution(hole_profile))
	generator = intersection(generator_body,
		grooves(hole, 1.5*height, alignment=0).flip(),
		)
	
	circulating_in = elliptic_circulating(rball, rballs).transform(0.5*height*Z)
	circulating_out = circulating_in.transform(rotate(pi,Y))
	
	axis = Axis(O,Z)
	joints = [
		Revolute(('in', 'ground'), axis),
		Revolute(('out', 'ground'), axis),
		Gear(('ground', 'spline'), 
			ratio = meshing.circle_teeth/meshing.flex_teeth, 
			centerline = (meshing.flex_max-meshing.flex_mean)*X, 
			axis = axis, 
			local = axis,
			),
		Revolute(('in', 'spline'), axis.transform((meshing.flex_max-meshing.flex_mean)*X), axis),
		]
	kin = Kinematic(joints, ground='ground', content={
		'ground': crown_in,
		'spline': flex,
		'in': [generator, circulating_in, circulating_out],
		})
	
	return Solid(
		height = height,
		rext = rext,
		nteeth = nteeth,
		crown_in = crown_in,
		crown_out = crown_out,
		flex = flex,
		generator = generator,
		circulating_in = circulating_in,
		circulating_out = circulating_out,
		)

if __name__ == '__madcad__':
	#settings.resolution = ('rad', 0.2)
	#settings.resolution = ('sqradm', 0.2)
	settings.resolution = ('sqradm', 0.8)
	
	gearbox = elliptic_double_crown(
		rext = 50,
		nteeth = 60,
		gap = 0.2,
		)
	
	#import os
	#folder = os.path.realpath(__file__+'/../elliptic-gearbox-double-{}-{}-{}'.format(gearbox.rext, gearbox.height, gearbox.nteeth))
	#os.mkdir(folder)
	#io.write(gearbox.flex, folder+'/flex.stl')
	#io.write(gearbox.crown_in, folder+'/crown_in.stl')
	#io.write(gearbox.crown_out, folder+'/crown_ou.stl')
	#io.write(gearbox.circulating_in.cage, folder+'/cage.stl')
	#io.write(gearbox.generator, folder+'/generator.stl')
	