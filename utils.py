import os
from collections import Counter

from madcad import *
from madcad.joints import *
from madcad.scheme import *


__all__ = ['export', 'screw_slot', 'bolt_slot', 'circular_screwing']


def export(model, path, dimensions):
	format = ''
	for dimension in dimensions:
		if isinstance(dimension, (int, str)):
			format += '-{}'
		elif isinstance(dimension, float):
			format += '-{:g}'
		else:
			raise ValueError('invalid dimension type: {}'.format(type(dimension)))
	folder = os.path.abspath(path + format.format(*dimensions))

	def remove(path):
		if os.path.isdir(path):
			for sub in os.listdir(path):
				remove('{}/{}'.format(path, sub))
			os.rmdir(path)
		elif os.path.exists(path):
			os.remove(path)
	
	counts = Counter()
	def count(model):
		if isinstance(model, Mesh):
			counts[id(model)] += 1
		elif isinstance(model, (Solid, dict)):
			for key, value in model.items():
				count(value)
		elif isinstance(model, (list, tuple)):
			for key, value in enumerate(model):
				count(value)
	memo = {}
	def write(model, path):
		if id(model) in memo:
			return
		memo[id(model)] = path
		# print('write', repr(model))
		if isinstance(model, Mesh):
			os.makedirs(os.path.abspath(path+'/..'), exist_ok=True)
			filename = '{}.stl'.format(path)
			print('exporting', filename)
			io.write(model, filename)
		elif isinstance(model, (Solid, dict)):
			for key, value in model.items():
				write(value, '{}/{}'.format(path, key))
		elif isinstance(model, (list, tuple)):
			for key, value in enumerate(model):
				write(value, '{}/{}'.format(path, key))
	
	remove(folder)
	count(model)
	write(model, folder)
	
	bom = open('{}/bom.csv'.format(folder), 'w')
	line = '{}\t{}\n'
	bom.write(line.format('part'.ljust(40), 'number'))
	for key in memo:
		if counts[key]:
			bom.write(line.format(memo[key][len(folder)+1:].ljust(40), counts[key]))


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
		profile.append(o + rslot*x + expand*z)
		profile.append(o + rslot*x)
	print('   gen', hole, screw, hole or screw)
	if hole or screw:
		print('   !!')
		if flat:
			profile.append(o + 0.5*dscrew*(x-z) - (rslot-dscrew)*z)
			hole = max(hole, dot(o-profile[-1], z))
		else:
			profile.append(o + 0.5*dscrew*x)
		profile.append(o + 0.5*dscrew*x - hole*z)
		profile.append(o + 0.4*dscrew*x - (hole+0.1*dscrew)*z)
		profile.append(o + 0.4*dscrew*x - (hole+screw)*z)
		profile.append(o - (hole+screw+0.4*dscrew)*z)
		print('   generated hole', hole, screw, len(profile))
	else:
		print('   problem')
		profile.append(o)
	print('  screw', repr(profile), expand, hole, screw)
	return revolution(wire(profile).segmented(), Axis(o,-z)).finish()

def bolt_slot(a: vec3, b: vec3, dscrew: float, rslot=None, hole=True, expanda=True, expandb=True) -> Mesh:
	''' bolt shape for a screw
		musch like `screw_slot()` but with two endings
		
		Parameters:
			a:  position of screw head
			b:  position of nut
			dscrew:  the screw diameter
			rslot:   the screw head slot radius
			hole:    enabled the cylindric hole between the head and nut slots
			expanda, expandb:
				- if `True`, enables slot sides on tip `a` or `b`
				- if `float`, it is the slot side height
				
		Example:
			
			>>> a, b = vec3(...), vec3(...)
			>>> bolts = bolt(a, b, 3)
			>>> part_hole = bolt_slot(a, b, 3)
	'''
	if not rslot:	rslot = 1.3*dscrew
	x,y,z = dirbase(normalize(a-b))

	def one_slot(o,x,z, expand):
		profile = []
		if expand:
			if isinstance(expand, bool):		expand = 2*rslot
			profile.append(o + rslot*x + expand*z)
			profile.append(o + rslot*x)
		if hole:
			profile.append(o + 0.5*dscrew*x)
		else:
			profile.append(o)
		return wire(profile).segmented()

	return revolution(
			one_slot(a,x,z, expanda) + one_slot(b,x,-z, expandb).flip(),
			Axis(a,-z),
			).finish()

def circular_screwing(axis, radius, height, dscrew, diameters:int=1, div:int=8, hold:float=0, nuts=True, screws=False, expand=True) -> '(Mesh, list)':
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
			nuts: 
				true if this screwing has screws and bolts on the other side.
				- if true, all screws are longer than `height`, and additional screws of maximum length `height` are added holding only on length `hold` 
				- if false, all screws are length height but only hold on length `hold`
			screws: enable to have screws in the final result
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
	expand = expand and diameters <= 1
	for i in range(diameters):
		d = enlarge*stfloor(0.8**i * dscrew)
		diameters_list.append(d)
		if nuts:
			slot = bolt_slot(a-gap, b+gap, 
						dscrew=d, expanda=expand, expandb=expand)
		else:
			slot = screw_slot(Axis(a-gap, -z), 
						dscrew=d, hole=height-hold, screw=hold, expand=expand)
		holes += slot .transform(rotatearound(i*1.4*dscrew/radius, axis))
	holes = repeataround(holes, div, axis)

	if screws:
		if nuts:
			part = bolt(O, height*Z, dscrew)
		else:
			part = screw(dscrew, height)
		#screws = repeataround(screw, div, axis)
		screws = [part.transform(translate(p)*mat4(mat3(x,y,z)))  for p in regon(axis, radius, div)]
		
	else:
		screws = None
	screwing = Solid(
		interface = Solid(
			perimeter = Circle(axis, radius),
			div = div,
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
		screws = screws,
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
