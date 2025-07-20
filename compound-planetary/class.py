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

#settings.primitives['curve_resolution'] = ('sqradm', 0.4)
settings.primitives['curve_resolution'] = ('sqradm', 0.8)



class CompoundPlanetary:
	def __init__(self,
		rext,
		zcrown_out = 51,
		zcrown_in = 44 + 2*9,
		zplanet_in = 9,
		zplanet_out = 8,
		):

		# global parameters
		self.rext = rext
		self.zcrown_out = zcrown_out,
		self.zcrown_in = zcrown_in,
		self.zplanet_in = zplanet_in,
		self.zplanet_out = zplanet_out,
		self.height = stceil(0.2*rext)
		self.dscrew_out = stceil(rext*0.1)
		self.dscrew_in = stceil(rext*0.08)
		self.shell_thickness = stceil(0.03 * rext)
		self.nplanets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
		self.zsun = zcrown_in - 2*zplanet_in

		# the bearing will constrain the mounting of the design
		self.bearing = dict(
			height = stceil(rext*0.2),
			rext = stfloor(rext-dscrew_out*0.6),
			rint = bearing_rext-bearing_height,
			)
			
		self.play = 0.2
		self.gap = 0.1*support_height
		self.hole = min(
			carrier.radius - planet_out.radius - 0.6*out_step - shell_thickness, 
			sun.radius - 0.5*in_step - shell_thickness,
			)
	
	def simplified(self,):
		return self.exterior(resolution=('sqradm', self.rext*0.1))
	
	def detailed(self, resolution=None):
		return Kinematic(self.joints(), ground='crown_in', content=self.assembly(resolution))
	
	
	def guides(self):
		# out_step is computed such as the out crown fit in the bearing out ring
		out_step = mix(self.bearing.rint, self.bearing.rext, 0.7) / (1 + zcrown_out/(2*pi))
		# in_step is such as the gearbox can be assembled
		in_step = out_step * (zcrown_out - zplanet_out) / (zcrown_in - zplanet_in)
		
		axis = Axis(O,Z)
		bearing_center = +0.7*self.bearing.height*Z

		# define marks in the assembly
		support_height = max(stceil(0.9*min(in_step, out_step)), 0.2*height)
		out_center = bearing_center + self.bearing.height*0.6*Z + 1*shell_thickness*Z + support_height*Z + height*0.5*Z
		in_center = out_center + height*Z + support_height*Z

		# place gears primitive circles
		carrier = Circle(Axis(in_center, Z), in_step*(zcrown_in-zplanet_in)/(2*pi))
		planet_out = Circle(Axis(out_center+carrier.radius*X,Z), out_step*zplanet_out/(2*pi))
		crown_out = Circle(Axis(out_center,Z), carrier.radius+planet_out.radius)
		planet_in = Circle(Axis(in_center + carrier.radius*X,Z), in_step*zplanet_in/(2*pi))
		crown_in = Circle(Axis(in_center,Z), carrier.radius + planet_in.radius)
		sun = Circle(Axis(in_center,Z), carrier.radius - planet_in.radius)

		gears_placeholder = [carrier, planet_out, crown_out, planet_in, crown_in, sun]
		
		return vars()
	
	def joints(self):
		joints = []
		for i, phase in enumerate(self.planet_phases()):
			p = rotate(phase,Z) * guides.planet_in.center
			positions = p, self.axis, self.axis.transform(p)
			joints.extend([
				Gear(('sun', 'planet_{}'.format(i)), -self.zsun/self.zplanet_in, *positions),
				Gear(('crown_in', 'planet_{}'.format(i)), self.zcrown_in/self.zplanet_in, *positions),
				Gear(('crown_out', 'planet_{}'.format(i)), self.zcrown_out/self.zplanet_out, *positions),
				])
		joints.append(Revolute(('sun', 'crown_in'), axis))

		return joints
	
	def annotations(self):
		return vars()

	def exterior(self, resolution=None):
		out = revolution(wire([
			crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X + gap*(Z-X),
			crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X,
			crown_out.center - height*0.5*Z - support_height*Z + (crown_out.radius + out_step*0.5)*X + shell_thickness*X,

			bearing_center + self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.3)*X,
			bearing_center + self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.)*X,
			bearing_center - self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.)*X,

			bearing_center - self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.3)*X,
			bearing_center - (self.bearing.height*0.7 + dscrew_out)*Z + mix(self.bearing.rint, self.bearing.rext, 0.3)*X,
			bearing_center - (self.bearing.height*0.7 + dscrew_out)*Z + (self.bearing.rint - dscrew_out*2)*X,
			bearing_center + self.bearing.height*0.4*Z + hole*X,
			bearing_center + self.bearing.height*0.7*Z + hole*X,
			bearing_center + self.bearing.height*0.7*Z + mix(self.bearing.rint, self.bearing.rext, 0.2)*X,

			crown_out.center - height*0.5*Z - support_height*Z + crown_out.radius*X,
			crown_out.center - height*0.5*Z + crown_out.radius*X,
			crown_out.center - height*0.5*Z + gap*Z + (crown_out.radius - in_step*0.6)*X,
			]).segmented())
		out = intersection(out, crown_out_teeth)
		
		in_coupling = grooves(stfloor(sun.radius*0.9), 1.3*height, alignment=1).flip().transform(in_center - height*0.6*Z)
		
		in_coupling = union(in_coupling, revolution(wire([
			- sun.radius*0.3*X,
			+ sun.radius*0.*X,
			]) .transform(in_center - height*0.4*Z + sun.radius*X)))
		
		input = revolution(wire([
			in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X,
			in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X + shell_thickness*Z,

			in_center + height*0.5*Z + min(hole, sun.radius*0.8)*X + shell_thickness*Z,
			in_center - height*0.5*Z + min(hole, sun.radius*0.8)*X,

			in_center - height*0.5*Z + sun.radius*X + in_step*X,
			]).segmented().flip())
		input = intersection(input, in_coupling)

		shell = wire([
			guides.crown_in.center - self.height*0.5*Z + guides.crown_in.radius*X,
			
			guides.crown_in.center - self.height*0.5*Z + (guides.crown_out.radius + self.out_step*0.6 + 2*self.shell_thickness)*X,
			guides.bearing.center + self.bearing.height*0.7*Z + (guides.crown_out.radius + self.out_step*0.6 + 2*self.shell_thickness)*X,
			
			guides.bearing.center + self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.7)*X,
			guides.bearing.center + self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 1.)*X,
			guides.bearing.center - self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 1.)*X,
			guides.bearing.center - self.bearing.height*0.5*Z + mix(self.bearing.rint, self.bearing.rext, 0.7)*X,
			guides.bearing.center - self.bearing.height*0.7*Z + mix(self.bearing.rint, self.bearing.rext, 0.7)*X,

			guides.bearing.center - self.bearing.height*0.7*Z + (self.rext + 1.*self.dscrew_out)*X,
			guides.bearing.center + self.bearing.height*1*Z + (self.rext + 1.*self.dscrew_out)*X,

			guides.bearing.center + self.bearing.height*1*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
			guides.crown_in.center - self.height*0.5*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
			guides.crown_in.center + self.height*0.5*Z + (crown_in.radius + in_step*0.5 + shell_thickness)*X,
			
			guides.in_ext.center + (guides.in_ext.radius + 1.5*self.dscrew_in)*X + self.shell_thickness*(X+Z),
			guides.in_ext.center + (guides.in_ext.radius - 1.2*self.dscrew_in)*X + self.shell_thickness*Z,
			guides.in_ext.center + (guides.in_ext.radius - 1.2*self.dscrew_in)*X,
			guides.in_ext.center + (guides.in_ext.radius + 1.5*self.dscrew_in)*X,
			guides.in_center + (guides.in_ext.radius + 1.5*self.dscrew_in)*X,
			]).segmented().flip()
		filet(shell, [12, 11], ('width', 0.5*height))
		filet(shell, [10], ('width', 0.5*dscrew_out))
		filet(shell, [1], ('width', 0.5*height))

		# put screws to hold everything
		in_ext_screw = bolt(
			in_ext.center + in_ext.radius*X,
			in_ext.center + in_ext.radius*X + 1*dscrew_in*Z,
			dscrew_in,
			nutb=False)

		holes, bolts_out_ext = circular_screwing(
			Axis(out_ext.axis.origin + self.bearing.height*1.7*Z, -Z), 
			out_ext.radius, self.bearing.height*1.7, dscrew_out, 
			diameters=2, hold=self.bearing.height*0.8)
		shell = intersection(shell, holes)
		holes, bolts_out_int = circular_screwing(out_int.axis, 
			out_int.radius, self.bearing.height*1.7, dscrew_out, 
			diameters=2, hold=1.3*dscrew_out)
		out = intersection(out, holes)
		holes, bolts_in_ext = circular_screwing(in_ext.axis, in_ext.radius, dscrew_in, dscrew_in, diameters=2)
		shell = intersection(shell, holes)

		return Solid(
			shell = shell, 
			out = out, 
			input = input,
			bolts = bolts_in_ext + bolts_out_ext,
			)
	
	def assembly(self, resolution=None): 
		bearing = bearing(2*self.bearing.rint, 2*self.bearing.rext, self.bearing.height) .transform(bearing_center)
		exterior = self.exterior(resolution)
		planets = self.planets()
		planets_phases = self.planet_phases()

		in_ext = Circle(
			Axis(in_center + height*0.5*Z + shell_thickness*Z + 1.2*dscrew_in*Z, Z), 
			stfloor(carrier.radius + planet_in.radius - in_step*0.6 - 1.2*dscrew_in),
			)
		out_ext = Circle(axis, rext)
		out_int = Circle(axis.transform(-dscrew_out*Z), stfloor(self.bearing.rint - 1.2*dscrew_in))
		in_placeholder = [
			in_ext, Circle(in_ext.axis.transform(in_ext.radius*X), dscrew_in/2),
			Circle(in_ext.axis, hole), Circle(in_ext.axis.transform((hole-1.2*dscrew_in)*X), dscrew_in/2),
			]
		out_placeholder = [
			out_ext, Circle(axis.transform(rext*X), dscrew_out/2), 
			out_int, Circle(out_int.axis.transform(out_int.radius*X), dscrew_out/2),
			]

		# generate all gears teeth
		vangle = radians(20)
		hangle = radians(20)
		sun_teeth = inflate(hgear(in_step, zsun, height+2*gap, -hangle, offset=-0.1*in_step, asymetry=-0.1) .transform(rotate(pi/zsun-pi*zplanet_in/zsun, Z)) .transform(sun.center), -play)
		crown_in_teeth = inflate(hgear(in_step, zcrown_in, height+4*gap, hangle, offset=0.1*in_step, asymetry=0.1).flip() .transform(crown_in.center), -play)
		crown_out_teeth = inflate(hgear(out_step, zcrown_out, height+2*support_height, hangle, offset=0, asymetry=0.2).transform(crown_out.center).flip(), -play)
		planet_in_teeth = hgear(in_step, zplanet_in, height+2*gap, hangle, offset=0.1*in_step, asymetry=0.0).transform(planet_in.center) .option(color=color_gear)
		planet_out_teeth = hgear(out_step, zplanet_out, height+2*gap, hangle, offset=0, asymetry=0.2).transform(planet_out.center) .option(color=color_gear)
		
		# support prevending the planets from collapsing under the pressure put by the torque of out crown 
		rsupport = carrier.radius - planet_out.radius + 0.4*out_step - 2*play
		planets_support = wire([
			(support_height+0.5*height-gap)*Z + hole*X,
			(support_height+0.5*height-gap)*Z + rsupport*X,
			(0.5*height+2*gap)*Z + rsupport*X,
			(0.5*height+2*gap)*Z + (carrier.radius - planet_out.radius - out_step*0.5)*X,
			])
		planets_support = revolution(
					(planets_support + planets_support.transform(scaledir(Z,-1)).flip()) 
					.transform(crown_out.center).segmented().close())

		# shape the teeth side
		crown_out_teeth = union(crown_out_teeth, revolution(wire([
			- out_step*0.6*X - 2*gap*Z,
			+ out_step*0.6*X,
			]) .transform(out_center - height*0.5*Z + crown_out.radius*X)))
		crown_out_teeth = intersection(crown_out_teeth, revolution(wire([
			- out_step*0.8*X - gap*Z,
			+ out_step*0.8*X + gap*Z,
			]) .transform(out_center + height*0.5*Z + crown_out.radius*X)))

		crown_in_teeth = intersection(crown_in_teeth, revolution(wire([
			+ in_step*0.8*X - gap*Z,
			- in_step*0.6*X + gap*Z,
			]) .transform(in_center - height*0.5*Z + crown_in.radius*X)))
		crown_in_teeth = union(crown_in_teeth, revolution(wire([
			+ in_step*0.6*X - gap*Z,
			- in_step*0.8*X + 2*gap*Z,
			]) .transform(in_center + height*0.5*Z + crown_in.radius*X)))

		sun_teeth = union(sun_teeth, revolution(wire([
			+ in_step*0.8*X +2*gap*Z,
			- in_step*0.6*X -gap*Z,
			]) .transform(in_center + height*0.5*Z + sun.radius*X)))
		sun_teeth = intersection(sun_teeth, revolution(wire([
			+ in_step*0.6*X +gap*Z,
			- in_step*0.6*X -gap*Z,
			]) .transform(in_center - height*0.5*Z + sun.radius*X)))

		# shape the main parts
		out = exterior['out']
		bolts = exterior['bolts']
		input = intersection(exterior['input'], sun_teeth)
		shell = intersection(revolution(exterior['shell']), crown_in_teeth)

		# cut the main parts into assebmlable parts
		shell_in = intersection(shell, square(Axis(bearing_center + self.bearing.height*0.7*Z, -Z), 1.5*rext))
		shell_mid = intersection(shell, 
			square(Axis(bearing_center + self.bearing.height*0.7*Z, Z), 1.5*rext)
			+ square(Axis(bearing_center + self.bearing.height*0.2*Z, -Z), 1.5*rext)
			)
		shell_out = intersection(shell, square(Axis(bearing_center + self.bearing.height*0.2*Z, Z), 1.5*rext))

		out_receiver = intersection(out, square(Axis(bearing_center - self.bearing.height*0.499*Z, -Z), 2*self.bearing.rint))
		out_int = intersection(out, square(Axis(bearing_center - self.bearing.height*0.499*Z, Z), 2*self.bearing.rint))

		planets = self.planets()
		
		return {
			'crown_out': [out_receiver, out_int],
			'crown_in': [shell, bolts],
			'sun': input,
			'planets': planets,
			}
	
	def planet_phases(self):
		# place as much planets as possible, their phasing is constrained by the sun and and crown
		max_planets = floor(pi*(self.zcrown_out - self.zplanet_out) / (self.zplanet_out + pi))
		perimeter = 0
		phase = 0
		phases = [phase]
		sector = 2*pi/max_planets
		for i in range(self.zcrown_in):
			new = pi/(self.zcrown_in - self.zplanet_in)*i
			if new > pi - sector:	
				break
			if new - phase > sector:
				phase = new
				phases.append(phase)

		phases += [phase+pi  for phase in phases]
		return phases

	def planet(self, resolution=None):
		# assemble the planet
		body = revolution(web([
			wire([
				self.crown_out.center - (0.5*self.height+self.support_height-self.gap)*Z + self.carrier.radius*X, 
				self.crown_out.center - (0.5*self.height+self.support_height-self.gap)*Z + self.rsupport*X, 
				self.crown_out.center - (0.5*self.height+self.gap)*Z + self.rsupport*X, 
				self.crown_out.center - (0.5*self.height-self.gap)*Z + (self.carrier.radius-self.planet_out.radius-0.5*self.out_step)*X,
				]),
			wire([
				self.crown_out.center + (0.5*self.height-self.gap)*Z + (self.carrier.radius-self.planet_out.radius-0.5*self.out_step)*X,
				self.crown_out.center + (0.5*self.height+self.gap)*Z + self.rsupport*X,
				self.crown_in.center - (0.5*self.height+self.gap)*Z + self.rsupport*X, 
				self.crown_in.center - (0.5*self.height-self.gap)*Z + (self.sun.radius-0.5*self.out_step)*X
				]),
			wire([
				self.crown_in.center + (0.5*self.height-self.gap)*Z + (self.sun.radius-0.5*self.out_step)*X, 
				self.crown_in.center + (0.5*self.height)*Z + self.rsupport*X,
				self.crown_in.center + (0.5*self.height)*Z + self.carrier.radius*X,
				]), 
			]).segmented().flip(), self.planet_in.axis)
		planet = intersection(body, self.planet_in_teeth + self.planet_out_teeth)
		return planet.option(color=color_gear)

	def planets(self):
		planets_phases = self.planet_phases()
		# rework planets to have the appropriate dephasing between in and out
		planets = []
		for i, phase in enumerate(planets_phases):
			in_half = pi/self.zplanet_in
			out_half = pi/self.zplanet_out
			in_transform = rotatearound((-phase*self.zcrown_in/self.zplanet_in - in_half) % (2*in_half) + in_half, self.planet_in.axis)
			out_transform = rotatearound((-phase*self.zcrown_out/self.zplanet_out - out_half) % (2*out_half) + out_half, self.planet_out.axis)
			def t(p):
				if p.z > mix(self.in_center.z, self.out_center.z, 0.5):	
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
						size=0.5*self.zplanet_in*self.in_step/pi,
						)
						.transform(rotate(pi/2,Z))
						.transform(self.planet_in.center + self.height*0.5*Z), 
					self.zplanet_in*self.in_step*0.02*Z,
					alignment=0.5)
			labeled = intersection(planet.transform(t), label.flip()).finish()
			planets.append(labeled.transform(rotate(phase,Z)).option(color=color_gear))
			
		return planets
		
	def to_stl(self, folder, assembly=None):
		if not assembly:
			assembly = self.assembly(resolution('sqradm', 0.01*self.rext))
		for i, planet in enumerate(assembly.planets):
			io.write(planets[i], f'{folder}/planet-{i}.stl')
		io.write(input, f'{folder}/input.stl')
		io.write(out_receiver, f'{folder}/out_receiver.stl')
		io.write(planets_support, f'{folder}/planets_support.stl')
		io.write(out_int, f'{folder}/out_int.stl')
		io.write(shell_in, f'{folder}/shell_in.stl')
		io.write(shell_out, f'{folder}/shell_out.stl')
		io.write(shell_mid, f'{folder}/shell_mid.stl')


if __name__ == '__main__':
	planetary = CompoundPlanetary(
		zcrown_out = 51,
		zcrown_in = 44 + 2*9,
		zplanet_in = 9,
		zplanet_out = 8,
		)
	obj = planetary.detailed()
	planetary.to_stl('/tmp/compound-planetary')
