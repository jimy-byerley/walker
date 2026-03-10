from madcad import *
from madcad.scheme import *

color_electronics = vec3(0.1, 0.5, 0.2)*0.5

@cachefunc
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
		

@cachefunc
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


@cachefunc
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

@cachefunc
def magnet_square(width, height):
	body = brick(size=vec3(width, width, height))
	filet(body, body.frontiers(), width=0.1*width, resolution=('div', 2))
	return Solid(
		width = width,
		height = height,
		body = body,
		annotations = note_bounds(body.box()),
		)

@cachefunc
def esp32_lite():
	width = 26
	length = 49
	thickness = 1.5
	dscrew = 2.5
	pin = 33/13
	npins = 13
	outline = parallelogram(width*X, length*Y, alignment=vec2(0.5, 0), fill=False)
	holes = Circle(Axis(-(width*0.5 - dscrew)*X + (length - dscrew)*Y, -Z), dscrew/2)
	chamfer(outline, [2,3], width=dscrew)
	board = extrusion(flatsurface(web([outline, holes])), -thickness*Z)
	gpios = (
		parallelogram(pin*X, pin*npins*Y, (2*pin+thickness)*Z, origin=-thickness/2*Z + (10 + pin/2)*Y + (width - pin)/2*X, alignment=vec3(0.5, 0, 0.5))
		+ parallelogram(pin*X, pin*npins*Y, (2*pin+thickness)*Z, origin=-thickness/2*Z + (10 + pin/2)*Y + -(width - pin)/2*X, alignment=vec3(0.5, 0, 0.5))
		)
	usb = parallelogram(9*X, 6.5*Y, 3.8*Z, origin=5.5*X, alignment=vec3(0.5, 0, 0))
	power = parallelogram(8*X, 6.5*Y, 5.5*Z, origin=-8*X, alignment=vec3(0.5, 0, 0))
	return Solid(
		width = width,
		length = length,
		dscrew = dscrew,
		outline = outline,
		board = board.option(color=color_electronics),
		connections = dict(
			gpios = gpios,
			usb = usb,
			power = power,
			),
		annotations = dict(
			width = note_distance(-width/2*X, width/2*X, offset=-width/2*Y),
			length = note_distance(O, length*Y, offset=width*X),
			spec = note_leading(O, text='ESP32 lite', offset=-width*Y + width*Z),
			),
		)

def power_pin(slots=1):
	return parallelogram(5*slots*X, 7.5*Y, 10*Z)

@cachefunc
def mks_dualfoc(controller=False):
	width = 40
	length = 56
	dscrew = 3
	thickness = 1.5
	pin = 33/13
	npins_sensors = 9
	npins_controller = 13
	width_controller = 26
	outline = parallelogram(width*X, length*Y, alignment=vec2(0.5, 0), fill=False)
	holes = web([
		Circle(Axis(-(width*0.5 - dscrew)*X + dscrew*Y, -Z), dscrew/2),
		Circle(Axis(+(width*0.5 - dscrew)*X + dscrew*Y, -Z), dscrew/2),
		Circle(Axis(-(width*0.5 - dscrew)*X + (length - dscrew)*Y, -Z), dscrew/2),
		Circle(Axis(+(width*0.5 - dscrew)*X + (length - dscrew)*Y, -Z), dscrew/2),
		])
	chamfer(outline, [0,1,2,3], width=dscrew)
	board = extrusion(flatsurface(web([outline, holes])), -thickness*Z)
	connections_power = (
		power_pin(2) .transform(translate(30*Y + width/2*X) * rotate(pi/2, Z))
		+ power_pin(6) .transform(translate(50*Y - width/2*X) * rotate(-pi/2, Z))
		)
	connections_sensors = parallelogram(npins_sensors*pin*X, (pin*2+9)*Y, pin*2*Z, origin=pin*2*Y, alignment=vec3(0.5, 1, 0))
	connections_controller = (
		parallelogram(pin*X, pin*npins_controller*Y, 10*Z, origin=-thickness*Z + (15 + pin/2)*Y + (width_controller - pin)/2*X, alignment=vec3(0.5, 0, 1))
		+ parallelogram(pin*X, pin*npins_controller*Y, 10*Z, origin=-thickness*Z + (15 + pin/2)*Y + -(width_controller - pin)/2*X, alignment=vec3(0.5, 0, 1))
		)
	if controller:
		controller = esp32_lite().transform(translate(-14*Z + 5*Y) * rotate(pi,Y))
	return Solid(
		width = width,
		length = length,
		dscrew = dscrew,
		outline = outline,
		board = board.option(color=color_electronics),
		connections = dict(
			power = connections_power, 
			sensors = connections_sensors,
			controller = connections_controller,
			),
		controller = controller,
		annotations = dict(
			width = note_distance(-width/2*X, width/2*X, offset=-width/2*Y),
			length = note_distance(O, length*Y, offset=width*X),
			spec = note_leading(O, text='MKS dualfoc v3', offset=-width*Y + width*Z),
			),
		)

if __name__ == '__madcad__':
	absolute_encoder = sensor_color()
	rotor_encoder = sensor_magnetic()
	magnet1 = magnet_round(5, 4)
	magnet2 = magnet_square(10, 5)
	controller = esp32_lite()
	driver = mks_dualfoc(controller=True)
