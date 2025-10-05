from madcad import *
from madcad.scheme import *

color_electronics = vec3(0.1, 0.5, 0.2)*0.5

def sensor_color():
	width = 20
	dscrew = 2.5
	spacing = (width - 18 + dscrew)/2
	pin = 3
	thickness = 1.2
	holes = [
		Circle(Axis((width/2-spacing)*-X + spacing*Y, Z), dscrew*0.5),
		Circle(Axis((width/2-spacing)*+X + spacing*Y, Z), dscrew*0.5),
		]
	outline = wire([
		width*0.5*+X,
		width*0.5*-X,
		width*0.5*-X + width*Y,
		width*0.5*+X + width*Y,
		]).close().segmented()
	chamfer(outline, outline.indices, width=dscrew)

	board = extrusion(flatsurface(web(outline) + web(holes)), Z*thickness)
	sensor = brick(center=9.5*Y, size=vec3(2, 2.4, 0.65), alignment=vec3(0.5, 0.5, 0)) .transform(Z*thickness)
	led = brick(center=9.5*Y + 3*X, size=vec3(2, 3, 1.5), alignment=vec3(0.5, 0.5, 0)) .transform(Z*thickness)
	connections = brick(center=width*Y, size=vec3(width, 3, 3), alignment=vec3(0.5, 1, 0)) .transform(Z*thickness)
	return Solid(
		width = width,
		dscrew = dscrew,
		holes = holes,
		outline = outline,
		board = board.option(color=color_electronics),
		sensor = sensor,
		led = led,
		connections = connections,
		)
		

def sensor_magnetic():
	width = 15
	diameter = 18
	dscrew = 2
	spacing = (width-14+dscrew)/2
	thickness = 1.2
	magnet_distance = 3
	outline = union(
		web(Circle(Axis(O,Z), diameter/2)),
		parallelogram(width*X, 2*diameter*Y, alignment=0.5, fill=False),
		)
	holes = [
		Circle(Axis((width/2-spacing)*-X, Z), dscrew*0.5),
		Circle(Axis((width/2-spacing)*+X, Z), dscrew*0.5),
		]
	board = extrusion(flatsurface(outline.flip() + web(holes)), Z*thickness)
	sensor = brick(size=vec3(4.9, 3.9, 1.75), alignment=vec3(0.5, 0.5, 0)).transform(Z*thickness)
	connections = brick(size=vec3(8.5, 4, 1.5), alignment=vec3(0.5, 1, 0), center=diameter*0.5*Y).transform(Z*thickness)
	return Solid(
		width = width,
		radius = diameter/2,
		dscrew = dscrew,
		holes = holes,
		outline = outline,
		magnet = Axis((thickness + magnet_distance + sensor.box().size.z)*Z, Z),
		board = board.option(color=color_electronics),
		sensor = sensor,
		connections = connections,
		)


def magnet_round(diameter, height):
	body = cylinder(-height/2*Z, height/2*Z, 5/2)
	filet(body, body.frontiers(), width=0.1*diameter, resolution=('div', 2))
	return Solid(
		diameter = diameter, 
		height = height,
		body = body,
		annotations = Solid(
			height = note_distance(-height/2*Z, +height/2*Z, offset=diameter*X),
			diameter = note_leading(height/2*Z + diameter*0.5*X, offset=diameter*0.5*(X+Z), text=f'ø {diameter}'),
			),
		)

def magnet_square(width, height):
	body = brick(size=vec3(width, width, height))
	filet(body, body.frontiers(), width=0.1*width, resolution=('div', 2))
	return Solid(
		width = width,
		height = height,
		body = body,
		annotations = note_bounds(body.box()),
		)


absolute_encoder = sensor_color()
rotor_encoder = sensor_magnetic()
magnet1 = magnet_round(5, 4)
magnet2 = magnet_square(10, 5)