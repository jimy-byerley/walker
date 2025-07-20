from madcad import *
from madcad.joints import *

tip = revolution(Softened([
		vec3(0.05116, 1.266e-07, -1.062),
		vec3(0.3262, 1.166e-07, -0.9785),
		vec3(0.614, 3.583e-08, -0.3006),
		vec3(0.518, 2.058e-08, -0.1727),
		vec3(0.25, 0., 0.25),
		])).flip().transform(-0.4*Z)
nail = union(
	difference(tip, icosphere(-0.6*Z-X, 0.8)),
	union(icosphere(0*Z, 0.4) + icosphere(1*Z, 0.3),
		cylinder(-1*Z, 1*Z, 0.15)),
	)
legdir = normalize(3*Z+X)

joints = [
	Ball(('leg', 'bot'), 0*Z),
	Ball(('leg', 'top'), 1*Z),
	Revolute(('leg', 'foreedge'), Axis(3*legdir, Y)),
	Revolute(('leg', 'backedge'), Axis(5*legdir, Y)),
	]
solids = {}
for i,t in enumerate(linrange(0, 2*pi, div=2)):
	r = rotate(t,Z)
	p = r * 1*X
	joints.extend([
		Ball(('top', 'nail-{}'.format(i)), p+1*Z),
		Ball(('bot', 'nail-{}'.format(i)), p+0*Z),
		])
	solids['nail-{}'.format(i)] = nail.transform(translate(p) * r)
kin = Kinematic(joints, content=solids, ground='bot')

def coilspring_conical(length, d1=None, d2=None, thickness=None, solid=True):
	''' Return a Mesh model of a croilspring meant for use in compression
	
		Parameters:
			length:     the distance between its two ends
			d:          the exterior diameter (the coilspring can fit in a cylinder of that diameter)
			thickness:  the wire diameter of the spring (useless if solid is disabled)
			solid:      disable it to get only the tube path of the coil, and have a `Wire` as return value
	'''
	if not d1:			d1 = length*0.4
	if not d2:            d2 = length*0.2
	if not thickness:	thickness = d*0.1
	r1 = d1/2 - thickness		# coil radius
	r2 = d2/2 - thickness		# coil radius
	e = min(r1, r2)					# coil step
	div = settings.curve_resolution(mix(d1, d2, 0.5)*pi, 2*pi)
	step = 2*pi/(div+1)
	
	t = 0
	
	t0, z0 = t, -0.5*length
	top = []
	for t in linrange(t0, t0 + 2*pi, step):
		top.append( vec3(r1*cos(t), r1*sin(t), z0 + (t-t0)/(2*pi) * thickness) )
	
	t0, z0 = t+step, -0.5*length + 1*thickness
	coil = []
	for x in linrange(0, 1, step / (2*pi * (length-4*thickness) / e)):
		t = t0 + 2*pi * (length-2*thickness) / e * x
		r = mix(r1, r2, x)
		coil.append( vec3(r*cos(t), r*sin(t), z0 + (t-t0)/(2*pi) * e) )
	
	t0, z0 = t+step, 0.5*length - 1*thickness
	bot = []
	for t in linrange(t0, t0 + 2*pi, step):
		bot.append( vec3(r2*cos(t), r2*sin(t), z0 + (t-t0)/(2*pi) * thickness) )
		
	path = Wire(top) + Wire(coil).qualify('spring') + Wire(bot)
	
	if not solid:
		return path
	
	return Solid(
			part=tube(
				flatsurface(Circle(
					(path[0],Y), 
					thickness/2, 
					resolution=('div',6)
					)) .flip(), 
				path,
				),
			axis=Axis(O, Z, interval=(-length/2, length/2)),
			top=0.5*length*Z,
			bottom=-0.5*length*Z,
			)

twister = coilspring_compression(0.6, 0.6, 0.05).transform(0.5*Z)
normalizer = coilspring_conical(1, 1.4, 0.6, 0.05).transform(1.5*Z)

def wave(start, stop, period:float, amplitude:vec3):
	pulse = distance(start, stop) / period * 2*pi
	return wire([
		mix(start, stop, x) + amplitude * sin(pulse*x)
		for x in linrange(0, 1, div=100)
		])

backwave = extrusion(wave(2*legdir+0.8*X, 5*legdir+1*X, 1, 0.1*X).flip(), 0.6*Y, alignment=0.5)
frontwave = extrusion(wave(2*legdir-0.8*X, 4*legdir-0.8*X, 1, -0.1*X), 0.6*Y, alignment=0.5)

kin.content['leg'] = [twister, normalizer, backwave, frontwave, icosphere(0*Z,0.5), icosphere(1*Z,0.4)]
kin.content['top'] = extrusion(flatsurface(convexoutline(web([
	Circle(Axis(p,Z), 0.5)  for p in regon(Axis(1*Z,Z), 0.9, 3) 
	]))), 0.2*Z, alignment=0.5).orient()
kin.content['bot'] = extrusion(flatsurface(convexoutline(web([
	Circle(Axis(p,Z), 0.7)  for p in regon(Axis(0*Z,Z), 0.8, 3) 
	]))), 0.2*Z).orient()
#kin.content['nail-0'] += icosphere(0*Z+X, 0.4) + icosphere(1*Z+X, 0.3) + cylinder(-1*Z+X, 1*Z+X, 0.15)