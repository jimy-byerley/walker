from madcad import *
from madcad.gear import gearprofile, rackprofile, minmax_radius
from madcad.joints import *
from madcad.scheme import *
import numpy as np
import scipy.optimize
from functools import reduce, partial
import operator


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

def ball_bearing(dint, dext=None, h=None, **kwargs):
	return Solid(
		rint = dint/2,
		rext = dext/2,
		height = h,
		placeholder = standard.bearing(dint, dext, h, **kwargs),
		annotations = [
			note_distance(O, dint/2*X),
			note_distance(dext/2*X - h/2*Z, dext/2*X + h/2*Z, offset=h*X),
			note_distance(dint/2*X + h/2*Z, dext/2*X + h/2*Z, offset=h*Z),
		])

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

def dual_crown_meshing(circle_radius, circle_teeth):
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
	
	base = wire(vec3(flex(flex_max, flex_min, phase),0)  for phase in phases).close()

	teeth = Solid()
	teeth.step = 2*pi*circle_radius / (circle_teeth-3)
	teeth.height = 0.3*teeth.step
	teeth.play = teeth.height*0.08
	tooth = (
		gearprofile(teeth.step, circle_teeth, 
			height=teeth.height,
			asymetry = 0,
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
			asymetry=teeth.height*0.17, 
			pressure_angle=radians(30),
			),
		circle_teeth, 
		rotate(2*pi/circle_teeth, Z),
		).close().option(color=vec3(0,0.5,1))
	teeth.crown_out = repeat(
		gearprofile(teeth.step, circle_teeth, 
			height=teeth.height, 
			asymetry=teeth.height*0.17,
			pressure_angle=radians(33),
			),
		flex_teeth, 
		rotate(2*pi/flex_teeth, Z),
		).close().option(color=vec3(0,1,0.5))
	circle = Circle(Axis(O,Z), circle_radius)

	return Solid(**vars())

def dual_crown_housing_guided(
		rext:float, 
		crown_in:Mesh, crown_out:Mesh, 
		dscrew_out:float=4, dscrew_in:float=None, 
		gap=0.2, shell_thickness=None, hole=None,
		out_clearance=0, in_clearance=0, 
		details=True):
	if not shell_thickness:
		shell_thickness = stceil(0.03 * rext)
	if not dscrew_in:
		dscrew_in = stceil(0.6 * dscrew_out)

	in_zmin = crown_in.box().min.z
	in_zmax = crown_in.box().max.z
	in_rmin, in_rmax = minmax_radius(crown_in.points)
	out_zmin = crown_out.box().min.z
	out_zmax = crown_out.box().max.z
	out_rmin, out_rmax = minmax_radius(crown_out.points)

	if hole is None:
		hole = min(in_rmin, out_rmin)*0.5
	
	# the bearing will constrain the mounting of the design
	bearing_height = stceil(rext*0.2)
	bearing_rext = stfloor(rext-dscrew_out*0.6)
	bearing_rint = bearing_rext-bearing_height
	bearing_center = (out_zmin - out_clearance - bearing_height*0.7)*Z
	bearing = ball_bearing(2*bearing_rint, 2*bearing_rext, bearing_height) .transform(bearing_center)

	# guiding primitives for input mechanical interface
	input = Solid(
		dscrew = stceil(rext*0.08),
		center = in_zmax*Z + shell_thickness*Z + 0.5*dscrew_in*Z,
		ext = Solid(),
		int = Solid(),
		)
	input.ext.perimeter = Circle(
		Axis(input.center,Z), 
		stceil(hole + 1.2*dscrew_in))
	input.ext.screw = Circle(
		input.ext.perimeter.axis.transform(input.ext.perimeter.radius*X), 
		input.dscrew/2)
	input.int = Circle(
		Axis(input.center,Z), 
		hole)
	
	# guiding primitives for output mechanical interface
	axis = Axis(O,Z)
	output = Solid(
		dscrew = stceil(rext*0.1),
		ext = Solid(),
		int = Solid(),
		)
	output.ext.perimeter = Circle(
		axis.transform(bearing_center*Z - bearing_height*0.7*Z), 
		rext)
	output.ext.screw = Circle(
		output.ext.perimeter.axis.transform(output.ext.perimeter.radius*X), 
		output.dscrew/2)
	output.int.perimeter = Circle(
		output.ext.perimeter.axis.transform(-dscrew_out*Z), 
		stfloor(bearing_rint - 1.2*dscrew_in))
	output.int.screw = Circle(
		output.int.perimeter.axis.transform(output.int.perimeter.radius*X), 
		output.dscrew/2)	


	# put screws to hold everything
	bolts = Solid(
		output = Solid(),
		input = Solid(),
		)
	holes = Solid(
		output = Solid(),
		input = Solid(),
		)
	holes.output.ext, bolts.output.ext = circular_screwing(
		output.ext.perimeter.axis.transform(bearing_height*1.7*Z).flip(), 
		output.ext.perimeter.radius, 
		bearing_height*1.7, 
		dscrew_out, 
		diameters=2, hold=bearing_height*0.8)
	holes.output.int, bolts.output.int = circular_screwing(
		output.int.perimeter.axis, 
		output.int.perimeter.radius, 
		bearing_height*1.7, 
		dscrew_out, 
		diameters=2, hold=1.3*dscrew_out)
	holes.input.ext, bolts.input.ext = circular_screwing(
		input.ext.perimeter.axis, 
		input.ext.perimeter.radius, 
		dscrew_in, 
		dscrew_in, 
		diameters=2)
	
	# put stops on teeth extremities
	crown_in = union(
		crown_in,
		revolution(wire([
			in_zmax*Z + mix(in_rmin, in_rmax, -0.1)*X,
			in_zmax*Z + mix(in_rmin, in_rmax, +1.1)*X - 0.3*(in_rmax-in_rmin)*Z,
			])).flip())
	crown_out = union(
		crown_out,
		revolution(wire([
			out_zmin*Z + mix(out_rmin, out_rmax, -0.1)*X,
			out_zmin*Z + mix(out_rmin, out_rmax, +1.1)*X + 0.3*(out_rmax-out_rmin)*Z,
			])))

	output_exterior = wire([
		bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
		bearing_center - (bearing_height*0.7 + dscrew_out)*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
		bearing_center - (bearing_height*0.7 + dscrew_out)*Z + (bearing_rint - dscrew_out*2)*X,
		bearing_center + bearing_height*0.4*Z + hole*X,
		bearing_center + bearing_height*0.7*Z + hole*X,
		bearing_center + bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.2)*X,
		]).segmented()
	if details:
		output_interior = wire([
			mix(out_rmin, out_rmax, 0.5)*X + out_zmin*Z - out_clearance*Z,
			mix(out_rmin, out_rmax, 0.5)*X + out_zmin*Z,
			out_rmin*0.98*X + out_zmin*Z + (out_rmax - out_rmin)*0.5*Z,
			out_rmin*0.98*X + out_zmax*Z - (out_rmax - out_rmin)*0.5*Z,
	
			out_rmax*X + out_zmax*Z - 0.1*(out_rmax - out_rmin)*Z,
			out_rmax*X + out_zmax*Z + shell_thickness*X - shell_thickness*Z,
			out_rmax*X + out_zmin*Z + shell_thickness*X - (out_clearance+shell_thickness)*Z,
	
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,
			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,
			]).segmented()
		output_body = intersection(
			revolution((output_interior + output_exterior).close()), 
			holes.output.int + crown_out)
	else:
		output_body = intersection(
			revolution(output_exterior), 
			holes.output.int)

	shell_outside = wire([
		bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
		bearing_center - bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.7)*X,

		bearing_center - bearing_height*0.7*Z + (rext + 1.*dscrew_out)*X,
		bearing_center + bearing_height*1*Z + (rext + 1.*dscrew_out)*X,

		(out_rmax+2*shell_thickness+gap)*X + bearing_center + bearing_height*1*Z,
		(out_rmax+2*shell_thickness+gap)*X + out_zmax*Z + shell_thickness*Z,
		(in_rmax+1*shell_thickness)*X + in_zmax*Z + shell_thickness*Z,
		
		input.ext.screw.center + (+ 1.5*dscrew_in)*X + shell_thickness*(X+Z),
		input.ext.screw.center + (- 1.2*dscrew_in)*X +shell_thickness*Z,
		input.ext.screw.center + (- 1.2*dscrew_in)*X,
		input.ext.screw.center + (+ 1.5*dscrew_in)*X,
		
		in_zmax*Z + in_clearance*Z + in_rmin*X,
		in_zmax*Z - gap*Z + in_rmin*X,
		]).segmented().flip()
	filet(shell_outside, [6, 5], width=1*dscrew_out)
	filet(shell_outside, [4], width=0.5*dscrew_out)
	
	if details:
		shell_inside = wire([
			(in_zmin+gap)*Z + in_rmin*X + (in_rmax-in_rmin)*0.5*Z,
			
			(in_zmin+gap)*Z + in_rmax*X,
			(out_zmax+gap)*Z + max(in_rmax, out_rmax)*X + (shell_thickness+gap)*X,
			out_zmin*Z + out_rmax*X + (shell_thickness+gap)*X - out_clearance*Z,
			
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
			]).segmented().flip()
		filet(shell_inside, [2], width=shell_thickness*3)
		
		shell = revolution(shell_outside + shell_inside)
		shell = intersection(shell, crown_in)
	else:
		shell = revolution(shell_outside)
	
	shell_body = intersection(shell, holes.output.ext + holes.input.ext)

	# cut the main parts into assebmlable parts
	if details:
		split_in = square(Axis(bearing_center + bearing_height*0.7*Z, -Z), 1.5*rext)
		split_out = square(Axis(bearing_center + bearing_height*0.2*Z, Z), 1.5*rext)
		shell = Solid(
			input = intersection(shell_body, split_in),
			mid = intersection(shell_body, split_in.flip() + split_out.flip()),
			output = intersection(shell_body, split_out),
			bearing = bearing,
			)
		split = square(Axis(bearing_center - bearing_height*0.2*Z, Z), 2*bearing_rint)
		output.int.crown = intersection(output_body, split.flip())
		output.int.front = intersection(output_body, split)
	else:
		shell = Solid(
			body = shell_body,
			bearing = bearing,
			)
		output.int.body = output_body

	output.int.bolts = bolts.output.int[1:]
	shell.bolts = bolts.output.ext[1:]

	return Solid(
		crown_in = shell,
		crown_out = output.int,
		)

def dual_crown_housing_free(rext:float, crown_in:Mesh, crown_out:Mesh, dscrew:float=4, gap=0.2):
	interface = Solid(dscrew = dscrew)
	interface.perimeter = Circle(Axis(interface.dscrew*Z,Z), rext)
	interface.screw = Circle(interface.perimeter.axis.transform(interface.perimeter.radius*X), interface.dscrew/2)
	
	rmin, rmax = minmax_radius(crown_in.points)
	zmax = crown_out.box().max.z

	holes, bolts = circular_screwing(
		interface.perimeter.axis, 
		interface.perimeter.radius, 
		zmax, 
		interface.dscrew, 
		diameters=3)
	
	crown_body = revolution(wire([
		mix(rmin, rmax, -0.1)*X + 0.5*(rmax-rmin)*Z + gap*Z,
		mix(rmin, rmax, 1.1)*X + gap*Z,
		mix(rmin, rmax, 1.5)*X + gap*Z,
		interface.screw.center - 1.2*interface.dscrew*X,
		interface.screw.center + interface.dscrew*X,
		interface.screw.center + interface.dscrew*X + (zmax-interface.dscrew)*Z,
		mix(rmin, rmax, 1.1)*X + zmax*Z,
		mix(rmin, rmax, -0.1)*X - 0.5*(rmax-rmin)*Z + zmax*Z,
		]).segmented().flip())
	crown_body = intersection(crown_body, holes)
	crown_out_body = intersection(
		crown_body,
		crown_out,
		)
	crown_in_body = intersection(
		crown_body.transform(scaledir(Z,-1)).flip(),
		crown_in,
		)
	return Solid(
		crown_in = crown_in_body,
		crown_out = crown_out_body,
		)


def strainwave_circulating(rball, rballs, nballs=None):
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
	return wire([vec3(rballs + rball*cos(t), 0, 1.08*rball*sin(t))
		for t in linrange(start, stop, div=10)])


@cachefunc
def strainwave_dual_crown(rext, nteeth, height=None, thickness = 0.9, rball = 3, dscrew = None, guided=True, details=False):
	if height is None:
		height = stceil(0.2*rext) # special case for cage height
		cage_height = rball*2.2
		if height/2 < cage_height:
			height = stceil(height/2 + cage_height)
	if dscrew is None:
		dscrew_out = stceil(rext*0.1)
		dscrew_in = stceil(rext*0.08)

	if guided:
		axial_play = 0.3  # axial play
		meshing = dual_crown_meshing(
			circle_radius = (rext-1.2*dscrew_out)*0.8, 
			circle_teeth = nteeth)	
		hole = stfloor((meshing.flex_min - 2*rball)*0.85)
		housing = dual_crown_housing_guided(
			rext,
			extrusion(meshing.teeth.crown_in, height*Z).flip(), 
			extrusion(meshing.teeth.crown_out, -height*Z), 
			dscrew_out=dscrew_out, 
			dscrew_in=dscrew_in,
			out_clearance = axial_play,
			hole = hole,
			gap = 1.5,
			details = details,
			)
	else:
		axial_play = 0  # axial play
		meshing = dual_crown_meshing(
			circle_radius = (rext-1.2*dscrew_out)*0.85, 
			circle_teeth = nteeth)	
		hole = stfloor((meshing.flex_min - 2*rball)*0.85)
		housing = dual_crown_housing_free(
			rext,
			extrusion(meshing.teeth.crown_in, -height*Z), 
			extrusion(meshing.teeth.crown_out, height*Z).flip(), 
			dscrew_out,
			gap = 0.2,
			details = details,
			)

	hole_profile = wire([
		height*Z + meshing.flex_max*X,
		height*Z + hole*X + height*0.1*X,
		height*+0.9*Z + hole*X,
		height*-0.9*Z + hole*X,
		-height*Z + hole*X + height*0.1*X,
		-height*Z + meshing.flex_max*X,
		]).segmented().flip()
	interface_in = intersection(
		revolution(hole_profile),
		grooves(hole, 1.5*height, alignment=0).flip(),
		)

	if details:
		rballs = rballs = meshing.flex_mean - rball - thickness
		ball_play = 0.05
		
		flex_body = revolution(wire([
			meshing.flex_mean*X + (height-axial_play)*Z + 3*meshing.teeth.height*X - meshing.teeth.height*Z,
			meshing.flex_mean*X + (height-axial_play)*Z,
			meshing.flex_mean*X + (height-axial_play)*Z - thickness*X,
			balls_guide_profile(rballs, rball+ball_play, radians(40), -radians(40)).transform((height-cage_height)*Z),
			meshing.flex_mean*X - thickness*X + (height*0.5-rball*1.3)*Z,
			meshing.flex_mean*X - thickness*X,
			])).flip()
		flex_body = flex_body + flex_body.transform(scaledir(Z,-1)).flip()
		flex = intersection(flex_body, extrusion(meshing.teeth.flex_initial, 2*height*Z, alignment=0.5)).option(color=settings.colors['gear'])
		
		generator_body = wire([
			(rballs-rball*1.1)*X + height*1.1*Z,
			balls_guide_profile(rballs, rball+ball_play, radians(-40)+pi, radians(90)+pi).transform((height-cage_height)*Z),
			(rballs+0.2*rball)*X + (height-cage_height-1.1*rball)*Z,
			(rballs+0.2*rball)*X,
			])
		
		generator_body = extrans(generator_body.transform(-meshing.flex_mean*X), (
			translate(vec3(meshing.flex(meshing.flex_max, meshing.flex_min, t),0)) 
			* mat4(normalize(quat(Y, vec3(meshing.dflex(meshing.flex_max, meshing.flex_min, t), 0))))
			for t in linrange(0, 2*pi, div=200)
			))
		generator_body += generator_body.transform(scaledir(Z,-1)).flip()
		generator_body.mergeclose()
		generator = intersection(generator_body, interface_in)
		
		circulating_in = strainwave_circulating(rball, rballs).transform((height - cage_height)*Z)
		circulating_out = circulating_in.transform(rotate(pi,Y))
	else:
		generator = interface_in
		
#	axis = Axis(O,Z)
#	joints = [
#		Revolute(('in', 'ground'), axis),
#		Revolute(('out', 'ground'), axis),
#		Gear(('ground', 'spline'), 
#			ratio = meshing.circle_teeth/meshing.flex_teeth, 
#			centerline = (meshing.flex_max-meshing.flex_mean)*X, 
#			axis = axis, 
#			local = axis,
#			),
#		Revolute(('in', 'spline'), axis.transform((meshing.flex_max-meshing.flex_mean)*X), axis),
#		]
#	kin = Kinematic(joints, ground='ground', content={
#		'ground': housing.crown_in,
#		'spline': flex,
#		'in': [generator, circulating_in, circulating_out],
#		})
	
	return Solid(
		height = height,
		rext = rext,
		guided = guided,
		nteeth = nteeth,
		crown_in = housing.crown_in,
		crown_out = housing.crown_out,
		flex = flex,
		generator = generator,
		circulating_in = circulating_in,
		circulating_out = circulating_out,
		)

strainwave = strainwave_dual_crown



if __name__ == '__madcad__':
	settings.resolution = ('sqradm', 0.2)
#	settings.resolution = ('sqradm', 0.4)
#	settings.resolution = ('sqradm', 0.8)
	
	gearbox = strainwave_dual_crown(
		rext = 50,
		nteeth = 60,
		guided = True,
		details = True,
		)
	
#	import os
#	folder = os.path.realpath(__file__+'/../strainwave-gearbox-double-f-{}-{}-{}'.format(
#		gearbox.rext, 
#		gearbox.height, 
#		gearbox.nteeth,
#		))
#	os.mkdir(folder)
#	io.write(gearbox.flex, folder+'/flex.stl')
#	io.write(gearbox.crown_in, folder+'/crown_in.stl')
#	io.write(gearbox.crown_out, folder+'/crown_ou.stl')
#	io.write(gearbox.circulating_in.cage, folder+'/cage.stl')
#	io.write(gearbox.generator, folder+'/generator.stl')
#	
#	import os
#	folder = os.path.realpath(__file__+'/../strainwave-gearbox-double-g-{}-{}-{}'.format(
#		gearbox.rext, 
#		gearbox.height, 
#		gearbox.nteeth,
#		))
#	print(folder)
#	os.mkdir(folder)
#	io.write(gearbox.flex, folder+'/flex.stl')
#	io.write(gearbox.crown_in.output, folder+'/crown_in_output.stl')
#	io.write(gearbox.crown_in.mid, folder+'/crown_in_mid.stl')
#	io.write(gearbox.crown_in.input, folder+'/crown_in_input.stl')
#	io.write(gearbox.crown_out.front, folder+'/crown_out_front.stl')
#	io.write(gearbox.crown_out.crown, folder+'/crown_out_crown.stl')
#	io.write(gearbox.circulating_in.cage, folder+'/cage.stl')
#	io.write(gearbox.generator, folder+'/generator.stl')
#	