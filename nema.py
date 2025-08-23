from madcad import *
from madcad.scheme import *

inch_to_millimeter = 25.4

def coupling_flat(diameter, length, depth=None):
	if depth is None:
		depth = stceil(diameter*0.3)
	return extrusion(wire([
		X*diameter,
		X*depth,
		X*depth + length*Z,
		X*diameter + length*Z,
		]).flip(), Y*diameter, alignment=0.5)

def coupling_key(diameter, length, depth=None):
	if depth is None:
		depth = stfloor(diameter*0.25)
	width = stfloor(diameter*0.5)
	return extrusion(flatsurface(convexoutline(web([
		Circle(Axis(O, X), width/2),
		Circle(Axis(O+Z*length, X), width/2),
		]))), X*diameter)
	

def square(width, spacing, dscrew, lscrew, dhub, hhub):
	screws = [
		Circle(Axis(p, Z), dscrew/2)
		for p in parallelogram(X*spacing, Y*spacing, align=0.5).points]
	hub = Circle(Axis(O+Z*hhub, Z), dhub/2)
	outline = parallelogram(X*width, Y*width, align=0.5, fill=False)
	chamfer(outline, list(range(4)), width=width*0.1)
	return Solid(
			width=width, spacing=spacing, dscrew=dscrew, lscrew=lscrew, dhub=dhub, hhub=hhub, 
			screws=screws, hub=hub, outline=outline,
			annotations=Solid(
				width = note_distance(-width/2*X, width/2*X, offset=-width*0.8*Y),
				spacing = note_distance(screws[0].center, screws[1].center, offset=0.3*width),
				dhub = note_radius(wire(hub), offset=width*0.5),
				dscrew = note_radius(wire(screws[0]), offset=width*0.5, text='4x M{}x{}'.format(dscrew, lscrew)),
			))

def round(width, spacing, dscrew, lscrew, dhub, hhub):
	screws = [
		Circle(Axis(p, Z), dscrew/2)
		for p in [-spacing/2*X, +spacing/2*X]]
	hub = Circle(Axis(O+Z*hhub, Z), dhub/2)
	outline = web(Circle(Axis(O, Z), width/2))
	return Solid(
			width=width, spacing=spacing, dscrew=dscrew, lscrew=lscrew, dhub=dhub, hhub=hhub, 
			screws=screws, hub=hub, outline=outline,
			annotations=Solid(
				width = note_distance(-width/2*X, +width/2*X, offset=width*0.8, text='ø {}'.format(width)),
				spacing = note_distance(-spacing/2*X, +spacing/2*X, offset=width*0.6),
				dhub = note_radius(wire(hub), offset=width*0.5),
				dscrew = note_radius(wire(screws[0]), offset=width*0.5, text='2x M{}x{}'.format(dscrew, lscrew)),
			))



squares = {
#	6: 14,
#	8: 20,
#	11: 28,
#	14: 35,
	17: square(width=42.2, spacing=31, dscrew=3, lscrew=4, dhub=22, hhub=2),
	23: square(width=57, spacing=47.1, dscrew=5, lscrew=4, dhub=38, hhub=1.5),
#	34: 86,
#	42: 110,
	}

rounds = {
	17: round(width=42, spacing=29, dscrew=4, lscrew=5, dhub=17.5, hhub=2),
	23: round(width=57, spacing=38.88, dscrew=4, lscrew=4, dhub=25, hhub=2),
	}

#@cachefunc
def motor(dim=17, length=None, dshaft=None, lshaft=None, coupling=None, round=False):
	if round:
		dim = rounds[dim]
	else:
		dim = squares[dim]
	if length is None:
		length = stceil(dim.width*1.1)
	if dshaft is None:
		dshaft = stceil(dim.width*0.15)
	if lshaft is None:
		lshaft = stceil(dim.width*0.5)
	body = union(
		extrusion(flatsurface(dim.outline), -length*Z),
		cylinder(O, dim.hub.center, dim.hub.radius),
		)
	holes = mesh.mesh([
		screw_slot(hole.axis.offset(dim.dscrew*0.1), hole.radius*2, screw=dim.lscrew, expand=False)
		for hole in dim.screws ])
	body = intersection(body, holes)
	shaft = cylinder(O, Z*lshaft, dshaft/2)
	chamfer(shaft, shaft.frontiers(0,2), width=stceil(dshaft*0.1))
	
	# TODO use this when the bug in uimadcad parsing is fixed
#	match coupling:
#		case 'flat':  slot = coupling_flat(dshaft, lshaft)
#		case 'key':   slot = coupling_key(dshaft, lshaft/2)
	
	if coupling:
		if coupling == 'flat':  slot = coupling_flat(dshaft, lshaft)
		elif coupling == 'key': slot = coupling_key(dshaft, lshaft/2)
		else:
			raise ValueError(f'unsupported coupling type: {coupling}')
		shaft = intersection(shaft, slot.transform(dshaft*Z))
	
	return Solid(
		interface=dim, 
		body=body, 
		shaft=shaft, 
		length=length,
		dshaft=dshaft, 
		lshaft=lshaft,
		annotations=Solid(
			dshaft = note_radius(shaft.group(0), text='ø {}'.format(dshaft)),
			length = note_distance(O, O-length*Z, offset=dim.width*0.7),
			lshaft = note_distance(O, O+lshaft*Z, offset=-dim.width*0.5),
		),
		)

m = motor(23, 53.5, 8, coupling='flat', round=False)
