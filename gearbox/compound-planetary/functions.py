from madcad import *
from madcad.gear import *
from madcad.joints import *
from madcad.text import *

def vprofile(radius, helix=radians(30)):
	helix = tan(helix)
	return [
		translate(t*height*Z) * rotate(sin(t*pi)/pi/radius*height*helix, Z)  
		for t in linrange(0, 1, div=10)]

def bearing(dint, dext=None, h=None, **kwargs):
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

# teeth generation functions
def vgear(step, teeth, height, angle, **kwargs):
	profile = repeataround(gearprofile(step, teeth, **kwargs))
	return mesh.mesh([
		extrans(profile, helix(height*0.5, step*teeth/(2*pi), angle)),
		extrans(profile, helix(-height*0.5, -step*teeth/(2*pi), angle)).flip(),
		]).mergegroups().finish()

def hgear(step, teeth, height, angle, **kwargs):
	print('gearprofile', step, teeth, kwargs)
	profile = repeataround(gearprofile(step, teeth, **kwargs), teeth)
	return (
		helix(profile, +0.5*height, angle, step*teeth/(2*pi))
		+ helix(profile, -0.5*height, angle, step*teeth/(2*pi)).flip()
		).mergegroups().finish()

def spurgear(step, teeth, height, **kwargs):
	return extrusion(repeataround(gearprofile(step, teeth, **kwargs)), height*Z, alignment=0.5)

color_gear = vec3(0.2, 0.3, 0.4)

annotations = {}


def compound_planetary(
	rext = 50,
	zcrown_out = 51,
	zcrown_in = 44 + 2*9,
	zplanet_in = 9,
	zplanet_out = 8,
	play = 0.2,
	):

	# global parameters
	height = stceil(0.2*rext)
	dscrew_out = stceil(rext*0.1)
	dscrew_in = stceil(rext*0.08)
	shell_thickness = stceil(0.03 * rext)
	nplanets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
	zsun = zcrown_in - 2*zplanet_in

	axis = Axis(O,Z)

	# the bearing will constrain the mounting of the design
	bearing_height = stceil(rext*0.2)
	bearing_rext = stfloor(rext-dscrew_out*0.6)
	bearing_rint = bearing_rext-bearing_height
	bearing_center = +0.7*bearing_height*Z
	bearing = bearing(2*bearing_rint, 2*bearing_rext, bearing_height) .transform(bearing_center)

	# out_step is computed such as the out crown fit in the bearing out ring
	out_step = mix(bearing_rint, bearing_rext, 0.7) / (1 + zcrown_out/(2*pi))
	# in_step is such as the gearbox can be assembled
	in_step = out_step * (zcrown_out - zplanet_out) / (zcrown_in - zplanet_in)

	# define marks in the assembly
	support_height = max(stceil(0.9*min(in_step, out_step)), 0.2*height)
	out_center = bearing_center + bearing_height*0.6*Z + 1*shell_thickness*Z + support_height*Z + height*0.5*Z
	in_center = out_center + height*Z + support_height*Z

	# place gears primitive circles
	carrier = Circle(Axis(in_center, Z), in_step*(zcrown_in-zplanet_in)/(2*pi))
	planet_out = Circle(Axis(out_center+carrier.radius*X,Z), out_step*zplanet_out/(2*pi))
	crown_out = Circle(Axis(out_center,Z), carrier.radius+planet_out.radius)
	planet_in = Circle(Axis(in_center + carrier.radius*X,Z), in_step*zplanet_in/(2*pi))
	crown_in = Circle(Axis(in_center,Z), carrier.radius + planet_in.radius)
	sun = Circle(Axis(in_center,Z), carrier.radius - planet_in.radius)

	gears_placeholder = [carrier, planet_out, crown_out, planet_in, crown_in, sun]

	gap = 0.1*support_height
	hole = min(
		carrier.radius - planet_out.radius - 0.6*out_step - shell_thickness, 
		sun.radius - 0.5*in_step - shell_thickness,
		)

	in_ext = Circle(
		Axis(in_center + height*0.5*Z + shell_thickness*Z + 1.2*dscrew_in*Z, Z), 
		stfloor(carrier.radius + planet_in.radius - in_step*0.6 - 1.2*dscrew_in),
		)
	out_ext = Circle(axis, rext)
	out_int = Circle(axis.transform(-dscrew_out*Z), stfloor(bearing_rint - 1.2*dscrew_in))
	in_placeholder = [
		in_ext, Circle(in_ext.axis.transform(in_ext.radius*X), dscrew_in/2),
		Circle(in_ext.axis, hole), Circle(in_ext.axis.transform((hole-1.2*dscrew_in)*X), dscrew_in/2),
		]
	out_placeholder = [
		out_ext, Circle(axis.transform(rext*X), dscrew_out/2), 
		out_int, Circle(out_int.axis.transform(out_int.radius*X), dscrew_out/2),
		]

	# generate all gears teeth
	def teeth():
		hangle = radians(20)
		sun = inflate(hgear(in_step, zsun, height+2*gap, -hangle, offset=-0.1*in_step, asymetry=-0.1) .transform(rotate(pi/zsun-pi*zplanet_in/zsun, Z)) .transform(sun.center), -play)
		crown_in = inflate(hgear(in_step, zcrown_in, height+4*gap, hangle, offset=0.1*in_step, asymetry=0.1).flip() .transform(crown_in.center), -play)
		crown_out = inflate(hgear(out_step, zcrown_out, height+2*support_height, hangle, offset=0, asymetry=0.2).transform(crown_out.center).flip(), -play)
		planet_in = hgear(in_step, zplanet_in, height+2*gap, hangle, offset=0.1*in_step, asymetry=0.0).transform(planet_in.center) .option(color=color_gear)
		planet_out = hgear(out_step, zplanet_out, height+2*gap, hangle, offset=0, asymetry=0.2).transform(planet_out.center) .option(color=color_gear)

		# shape the teeth side
		teeth = Solid()
		crown_out = union(crown_out, revolution(wire([
			- out_step*0.6*X - 2*gap*Z,
			+ out_step*0.6*X,
			]) .transform(out_center - height*0.5*Z + crown_out.radius*X)))
		teeth.crown_out = intersection(crown_out, revolution(wire([
			- out_step*0.8*X - gap*Z,
			+ out_step*0.8*X + gap*Z,
			]) .transform(out_center + height*0.5*Z + crown_out.radius*X)))

		teeth.crown_in = intersection(crown_in, revolution(wire([
			+ in_step*0.8*X - gap*Z,
			- in_step*0.6*X + gap*Z,
			]) .transform(in_center - height*0.5*Z + crown_in.radius*X)))
		teeth.crown_in = union(crown_in, revolution(wire([
			+ in_step*0.6*X - gap*Z,
			- in_step*0.8*X + 2*gap*Z,
			]) .transform(in_center + height*0.5*Z + crown_in.radius*X)))

		teeth.sun = union(sun, revolution(wire([
			+ in_step*0.8*X +2*gap*Z,
			- in_step*0.6*X -gap*Z,
			]) .transform(in_center + height*0.5*Z + sun.radius*X)))
		teeth.sun = intersection(sun, revolution(wire([
			+ in_step*0.6*X +gap*Z,
			- in_step*0.6*X -gap*Z,
			]) .transform(in_center - height*0.5*Z + sun.radius*X)))
		return teeth


	def planets_phases():
		# place as much planets as possible, their phasing is constrained by the sun and and crown
		max_planets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
		perimeter = 0
		phase = 0
		planets_phases = [phase]
		planet_sector = 2*pi/max_planets
		for i in range(zcrown_in):
			new = pi/(zcrown_in - zplanet_in)*i
			if new > pi - planet_sector:	
				break
			if new - phase > planet_sector:
				phase = new
				planets_phases.append(phase)

		planets_phases += [phase+pi  for phase in planets_phases]
		planets = [(
					planet_in_teeth.transform(rotatearound(-phase*zcrown_in/zplanet_in, planet_in.axis))
					+ planet_out_teeth.transform(rotatearound(-phase*zcrown_out/zplanet_out, planet_out.axis))
				).transform(rotate(phase,Z)).option(color=color_gear)
				for phase in planets_phases]
		return planets_phases


	# support prevending the planets from collapsing under the pressure put by the torque of out crown 
	def carrier():
		rsupport = carrier.radius - planet_out.radius + 0.4*out_step - 2*play
		planets_support = wire([
			(support_height+0.5*height-gap)*Z + hole*X,
			(support_height+0.5*height-gap)*Z + rsupport*X,
			(0.5*height+2*gap)*Z + rsupport*X,
			(0.5*height+2*gap)*Z + (carrier.radius - planet_out.radius - out_step*0.5)*X,
			])
		return revolution(
					(planets_support + planets_support.transform(scaledir(Z,-1)).flip()) 
					.transform(crown_out.center).segmented().close())

	# assemble the planet
	def planet():
		planet_body = revolution(web([
			wire([
				crown_out.center - (0.5*height+support_height-gap)*Z + carrier.radius*X, 
				crown_out.center - (0.5*height+support_height-gap)*Z + rsupport*X, 
				crown_out.center - (0.5*height+gap)*Z + rsupport*X, 
				crown_out.center - (0.5*height-gap)*Z + (carrier.radius-planet_out.radius-0.5*out_step)*X,
				]),
			wire([
				crown_out.center + (0.5*height-gap)*Z + (carrier.radius-planet_out.radius-0.5*out_step)*X,
				crown_out.center + (0.5*height+gap)*Z + rsupport*X,
				crown_in.center - (0.5*height+gap)*Z + rsupport*X, 
				crown_in.center - (0.5*height-gap)*Z + (sun.radius-0.5*out_step)*X
				]),
			wire([
				crown_in.center + (0.5*height-gap)*Z + (sun.radius-0.5*out_step)*X, 
				crown_in.center + (0.5*height)*Z + rsupport*X,
				crown_in.center + (0.5*height)*Z + carrier.radius*X,
				]), 
			]).segmented().flip(), planet_in.axis)
		return intersection(planet_body, planet_in_teeth + planet_out_teeth).option(color=color_gear)

	def screwing():
		# put screws to hold everything
		in_ext_screw = bolt(
			in_ext.center + in_ext.radius*X,
			in_ext.center + in_ext.radius*X + 1*dscrew_in*Z,
			dscrew_in,
			nutb=False)

		holes = Solid()
		screws = Solid()
		holes.out_ext, bolts.out_ext = circular_screwing(
			Axis(out_ext.axis.origin + bearing_height*1.7*Z, -Z), 
			out_ext.radius, bearing_height*1.7, dscrew_out, 
			diameters=2, hold=bearing_height*0.8)
		
		holes.out_int, bolts.out_int = circular_screwing(out_int.axis, 
			out_int.radius, bearing_height*1.7, dscrew_out, 
			diameters=2, hold=1.3*dscrew_out)
		
		holes.shell, bolts.in_ext = circular_screwing(in_ext.axis, in_ext.radius, dscrew_in, dscrew_in, diameters=2)
		return holes, bolts

	def input(teeth):
		in_coupling = grooves(stfloor(sun.radius*0.9), 1.3*height, alignment=1).flip().transform(in_center - height*0.6*Z)
		in_coupling = union(in_coupling, revolution(wire([
			- sun.radius*0.3*X,
			+ sun.radius*0.*X,
			]) .transform(in_center - height*0.4*Z + sun.radius*X)))
		# shape the main parts
		input = revolution(wire([
			in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X,
			in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X + shell_thickness*Z,

			in_center + height*0.5*Z + min(hole, sun.radius*0.8)*X + shell_thickness*Z,
			in_center - height*0.5*Z + min(hole, sun.radius*0.8)*X,

			in_center - height*0.5*Z + sun.radius*X + in_step*X,
			]).segmented().flip())
		input = intersection(input, (teeth.sun + in_coupling))
		return input

	def shell(teeth, holes, bolts):
		shell = wire([
			crown_in.center - height*0.5*Z + crown_in.radius*X,
			
			crown_in.center - height*0.5*Z + (crown_out.radius + out_step*0.6 + 2*shell_thickness)*X,
			bearing_center + bearing_height*0.7*Z + (crown_out.radius + out_step*0.6 + 2*shell_thickness)*X,
			
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
			bearing_center - bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.7)*X,

			bearing_center - bearing_height*0.7*Z + (rext + 1.*dscrew_out)*X,
			bearing_center + bearing_height*1*Z + (rext + 1.*dscrew_out)*X,

			bearing_center + bearing_height*1*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
			crown_in.center - height*0.5*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
			crown_in.center + height*0.5*Z + (crown_in.radius + in_step*0.5 + shell_thickness)*X,
			
			in_ext.center + (in_ext.radius + 1.5*dscrew_in)*X + shell_thickness*(X+Z),
			in_ext.center + (in_ext.radius - 1.2*dscrew_in)*X +shell_thickness*Z,
			in_ext.center + (in_ext.radius - 1.2*dscrew_in)*X,
			in_ext.center + (in_ext.radius + 1.5*dscrew_in)*X,
			in_center + (in_ext.radius + 1.5*dscrew_in)*X,
			]).segmented().flip()
		filet(shell, [12, 11], ('width', 0.5*height))
		filet(shell, [10], ('width', 0.5*dscrew_out))
		filet(shell, [1], ('width', 0.5*height))
		shell = intersection(revolution(shell), crown_in_teeth)
		shell = intersection(shell, holes.out_ext)
		shell = intersection(shell, holes.in_ext)
		
		split_input = square(Axis(bearing_center + bearing_height*0.7*Z, -Z), 1.5*rext)
		split_output = square(Axis(bearing_center + bearing_height*0.2*Z, Z), 1.5*rext)
		shell = Solid(
			input = intersection(shell, split_input),
			mid = intersection(shell, split_input.flip() + split_output.flip()),
			output = intersection(shell, split_output),
			bolts = bolts.out_ext,
			)
		return shell

	def output(teeth, holes, bolts):
		out = revolution(wire([
			crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X + gap*(Z-X),
			crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X,
			crown_out.center - height*0.5*Z - support_height*Z + (crown_out.radius + out_step*0.5)*X + shell_thickness*X,

			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
			bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,
			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,

			bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
			bearing_center - (bearing_height*0.7 + dscrew_out)*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
			bearing_center - (bearing_height*0.7 + dscrew_out)*Z + (bearing_rint - dscrew_out*2)*X,
			bearing_center + bearing_height*0.4*Z + hole*X,
			bearing_center + bearing_height*0.7*Z + hole*X,
			bearing_center + bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.2)*X,

			crown_out.center - height*0.5*Z - support_height*Z + crown_out.radius*X,
			crown_out.center - height*0.5*Z + crown_out.radius*X,
			crown_out.center - height*0.5*Z + gap*Z + (crown_out.radius - in_step*0.6)*X,
			]).segmented())
		out = intersection(out, teeth.crown_out)
		out = intersection(out, holes.out_int)
		# cut the main parts into assebmlable parts
		split = square(Axis(bearing_center - bearing_height*0.499*Z, -Z), 2*bearing_rint)
		return Solid(
			receiver = intersection(out, split),
			interior = intersection(out, split.flip()),
			bolts = bolts.out_int,
			)

	def planets():
		# rework planets to have the appropriate dephasing between in and out
		planets = [(
					planet_in_teeth.transform(rotatearound(-phase*zcrown_in/zplanet_in, planet_in.axis))
					+ planet_out_teeth.transform(rotatearound(-phase*zcrown_out/zplanet_out, planet_out.axis))
				).transform(rotate(phase,Z)).option(color=color_gear)
				for phase in planets_phases]
		planets = []
		for i, phase in enumerate(planets_phases):
			in_half = pi/zplanet_in
			out_half = pi/zplanet_out
			in_transform = rotatearound((-phase*zcrown_in/zplanet_in - in_half) % (2*in_half) + in_half, planet_in.axis)
			out_transform = rotatearound((-phase*zcrown_out/zplanet_out - out_half) % (2*out_half) + out_half, planet_out.axis)
			def t(p):
				if p.z > mix(in_center.z, out_center.z, 0.5):	
					return in_transform * p
				else: 
					return out_transform * p
			
			# put an index on each planet
			label = extrusion(
					text(str(i), 
						align=('center', 'center'), 
						fill=True, 
						font='PuristaBold',
						resolution=('rad', 0.4),
						size=0.5*zplanet_in*in_step/pi,
						)
						.transform(rotate(pi/2,Z))
						.transform(planet_in.center + height*0.5*Z), 
					zplanet_in*in_step*0.02*Z,
					alignment=0.5)
			labeled = intersection(planet.transform(t), label.flip()).finish()
			planets.append(labeled.transform(rotate(phase,Z)).option(color=color_gear))
		
		return planets

	def kinematic():
		joints = []
		for i, phase in enumerate(planets_phases):
			p = rotate(phase,Z) * planet_in.center
			joints.extend([
				Gear(('sun', 'planet_{}'.format(i)), -zsun/zplanet_in, p, axis, Axis(p,Z)),
				Gear(('crown_in', 'planet_{}'.format(i)), zcrown_in/zplanet_in, p, axis, Axis(p,Z)),
				Gear(('crown_out', 'planet_{}'.format(i)), zcrown_out/zplanet_out, p, axis, Axis(p,Z)),
				])
		joints.append(Revolute(('sun', 'crown_in'), axis))
		return Kinematic(joints, ground='crown_in')
		
	def assembly():
		teeth = teeth()
		holes, bolts = screwing()
		input = input(teeth, holes, bolts)
		output = output(teeth, holes, bolts)
		
		kin = kinematic()
		kin.content = {
			'crown_out': [shells(), standard.bearing(bearing)],
			'sun': input()
			}
		for planet in planets():
			kin.content['planet_{}'.format(i)] = planet[i]
		return kin
		
	def placeholder():
		return Solid(out_receiver(), shell_ext())
		
	if details:
		return assembly()
	else:
		return placeholder()


def print():
	for i, planet in enumerate(planets):
		io.write(planets[i], '/tmp/compound-planetary/planet-{}.stl'.format(i))
	io.write(input, '/tmp/compound-planetary/input.stl')
	io.write(out_receiver, '/tmp/compound-planetary/out_receiver.stl')
	io.write(planets_support, '/tmp/compound-planetary/planets_support.stl')
	io.write(out_int, '/tmp/compound-planetary/out_int.stl')
	io.write(shell_in, '/tmp/compound-planetary/shell_in.stl')
	io.write(shell_out, '/tmp/compound-planetary/shell_out.stl')
	io.write(shell_mid, '/tmp/compound-planetary/shell_mid.stl')


#settings.primitives['curve_resolution'] = ('sqradm', 0.4)
settings.primitives['curve_resolution'] = ('sqradm', 0.8)
result = compound_planetary(..., details=True)
