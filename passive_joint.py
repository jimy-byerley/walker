from madcad import *
from madcad.scheme import *
from copy import copy
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

def ball_bearing(dint, dext=None, h=None, **kwargs):
	return Solid(
		rint = dint/2,
		rext = dext/2,
		height = h,
		placeholder = standard.bearing(dint, dext, h, **kwargs).part,
		annotations = Solid(
			name = note_leading(dext/2*X + h/2*Z, h*2*(X+Z), text='{:g}x{:g}x{:g}\nball bearing'.format(dint, dext, h)),
			rint = note_distance(O, dint/2*X),
			height = note_distance(dext/2*X - h/2*Z, dext/2*X + h/2*Z, offset=h*X),
			width = note_distance(dint/2*X + h/2*Z, dext/2*X + h/2*Z, offset=h*Z),
		))

@cachefunc
def passive_joint(rext, dscrew, nscrew=8):
	print(rext, 4*dscrew)
	rint = stfloor(rext - 4*dscrew)
	bdint = stceil(2*(rint+0.7*dscrew))
	bdext = stfloor(2*(rext-0.7*dscrew))
	bearing = ball_bearing(
		bdint,
		bdext, 
		stceil((bdext-bdint)/2),
		)
	def outer(rext, rint):
		height = bearing.height*0.7
		profile = wire([
			rext*X + dscrew*X + height*Z,
			mix(bearing.rext, bearing.rint, 0.3)*X + height*Z,
			mix(bearing.rext, bearing.rint, 0.3)*X + bearing.height*0.5*Z,
			bearing.rext*X + bearing.height*0.5*Z,
			]).segmented()
		body = revolution((
			profile.flip()
			+ profile.transform(scaledir(Z, -1))
			).close())
		screwing = circular_screwing(
			Axis(-bearing.height*0.7*Z, Z), 
			radius = rext, 
			height = 1.4*bearing.height, 
			dscrew = dscrew,
			div = nscrew,
			diameters = 2)
		wrench_slot = brick(size=vec3(2*rext*cos(pi/nscrew/2) + dscrew, 3*rext, 2*bearing.height + 2*dscrew)) .transform(rotate(pi/nscrew, Z))
		part = intersection(body, screwing.holes + wrench_slot)
		split = square(Axis(O,Z), rext*1.5)
		return copy(screwing.interface).update(
			input = intersection(part, split),
			output = intersection(part, split.flip()),
			height = height,
			annotations = note_distance(O, height*Z, offset=-1.5*rext*X),
			)

	def inner(rext, rint):
		height = bearing.height*0.7 + dscrew
		profile = wire([
			rint*X - dscrew*X + height*Z,
			mix(bearing.rint, bearing.rext, 0.3)*X + height*Z,
			mix(bearing.rint, bearing.rext, 0.3)*X + bearing.height*0.5*Z,
			bearing.rint*X + bearing.height*0.5*Z,
			]).segmented()
		body = revolution((
			profile
			+ profile.flip().transform(scaledir(Z, -1))
			).close())
		screwing = circular_screwing(
			Axis(-bearing.height*0.7*Z - dscrew*Z, Z), 
			radius = rint, 
			height = 1.4*bearing.height + 2*dscrew, 
			dscrew = dscrew,
			div = nscrew,
			diameters = 2 if 5*dscrew < rint else 1)
		part = intersection(body, screwing.holes)
		split = square(Axis(bearing.height*0.499, Z), rext)
		return copy(screwing.interface).update(
			input = intersection(part, split),
			output = intersection(part, split.flip()),
			height = height,
			annotations = note_distance(O, height*Z, offset=-1.5*rext*X),
			)
	
	return Solid(
		outer = outer(rext, rint),
		inner = inner(rext, rint),
		bearing = bearing,
		)

if __name__ == '__madcad__':
#	settings.resolution = ('sqradm', 0.2)
#	settings.resolution = ('sqradm', 0.3)
	settings.resolution = ('sqradm', 0.8)

	j1 = passive_joint(rext=50, dscrew=5, nscrew=8)
	j2 = passive_joint(rext=30, dscrew=5, nscrew=4).transform(translate(50*Z))
