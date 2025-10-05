from madcad import *
from madcad.joints import *
from madcad.scheme import *

from gearbox.strainwave import strainwave_dual_crown, grooves, circular_screwing
from utils import *
import nema
import sensors



def joint():
	gearbox = strainwave_dual_crown(
			rext = 50,
			nteeth = 60,
			guided = True,
			details = True,
			)
	motor = nema.motor(17, 43, 5, 20, round=True, coupling='flat')
	absolute_encoder = ( sensors.sensor_color()
			.transform(translate(gearbox.rext*Y - 8*Z) * rotate(pi*0.6, X)) )
	rotor_encoder = sensors.sensor_magnetic()
	magnet = sensors.magnet_square(10, 5)
	
	rotor_encoder_clearance = rotor_encoder.magnet.origin.z
	rotor_top = gearbox.shell.input.perimeter.center
	magnet_spacing = motor.dshaft*1.5
	coupling_dscrew = 5
	
	motor = motor.place((Revolute, Axis(motor.lshaft*Z, Z), Axis(rotor_top, Z)))
	rotor_encoder = ( rotor_encoder
		.place((Revolute, Axis(O,-Z), Axis(rotor_top + rotor_encoder_clearance*Z, Z)))
		.transform(rotate(pi/4, Z)) )
	
	magnets = [
		magnet.transform(translate(rotor_top - magnet.width/2*Z + (magnet_spacing+magnet.height/2)*Y) * rotate(pi/2, X) ),
		magnet.transform(translate(rotor_top - magnet.width/2*Z - (magnet_spacing+magnet.height/2)*Y) * rotate(-pi/2, X) ),
		]
	
	def build_rotor():
		play = 0.8
		coupling_gearbox = union(
			intersection(
				inflate(grooves(gearbox.hole, gearbox.height*1.5, alignment=0).transform(play*Z), -play/2),
				revolution(wire([
					-gearbox.height*Z + gearbox.hole*1.06*X,
					gearbox.height*Z + gearbox.hole*1.06*X,
					gearbox.height*Z + gearbox.hole*0.9*X,
					rotor_top + rotor_encoder.radius*X,
					rotor_top,
					]).segmented().flip()),
				),
			revolution(wire([
				rotor_top - motor.lshaft*0.8*Z,
				rotor_top - motor.lshaft*0.8*Z + gearbox.hole*0.9*X,
				gearbox.height*0.2*Z + gearbox.hole*0.9*X,
				]).segmented().flip()),
			)
		
		coupling_screw = ( screw(coupling_dscrew, coupling_dscrew*2.5, head='HH')
			.place((Revolute, Axis(O,Z), Axis(gearbox.height/2*Z + motor.dshaft/2*X + coupling_dscrew*2.3*X, X))) )
		cut = wire([
			(motor.dshaft/2 + coupling_dscrew*2)*(X+Y)*2,
			(motor.dshaft/2 + coupling_dscrew*2)*(X+Y),
			(motor.dshaft/2 + coupling_dscrew*2)*(X-Y),
			(motor.dshaft/2 + coupling_dscrew*2)*(X-Y)*2,
			]).segmented()
		filet(cut, [1,2], width=coupling_dscrew)
		coupling_rotor = intersection(
			cylinder(
				gearbox.height/2*Z, 
				gearbox.height/2*Z + gearbox.hole*X, 
				coupling_dscrew/2).flip(),
			extrusion(cut.transform(rotor_top), -motor.lshaft*Z) 
			+ cylinder(rotor_top+play*Z, rotor_top-motor.lshaft*Z, coupling_dscrew/2).flip(),
			)
		def magnet_slot(magnet):
			magnet_bounds = magnet.body.transform(magnet.pose).box()
			return extrusion(
				convexoutline(web([
					Circle(Axis(magnet_bounds.map(vec3(0, 0.5, 0.5))-play*X, Z), magnet_bounds.size.y/2),
					Circle(Axis(magnet_bounds.map(vec3(1, 0.5, 0.5))+play*X, Z), magnet_bounds.size.y/2),
					])),
				motor.lshaft*3*Z,
				alignment=0.5).orient().flip()
		magnet_slots = mesh.mesh([ magnet_slot(magnet)  for magnet in magnets])
		holes = [
			cylinder(
				-gearbox.hole*0.4*X - motor.dshaft/2*X + rotor_top - motor.lshaft*Z, 
				-gearbox.hole*0.4*X - motor.dshaft/2*X + rotor_top + play*Z, 
				gearbox.hole*0.25).flip(),
		#	cylinder(
		#		-gearbox.hole*(0.35*X+0.5*Y) - motor.dshaft/2*X, 
		#		-gearbox.hole*(0.35*X+0.5*Y) - motor.dshaft/2*X + top + play*Z, 
		#		gearbox.hole*0.15).flip(),
		#	cylinder(
		#		-gearbox.hole*(0.35*X-0.5*Y) - motor.dshaft/2*X, 
		#		-gearbox.hole*(0.35*X-0.5*Y) - motor.dshaft/2*X + top + play*Z, 
		#		gearbox.hole*0.15).flip(),
			]
		coupling = intersection(
			coupling_gearbox, 
			coupling_rotor + magnet_slots.mergegroups().qualify('magnets') + mesh.mesh(holes),
			).finish()
#		f = coupling.frontiers(None, 'magnets')
#		edgecut(coupling, f, width=play)
		chamfer(coupling, coupling.frontiers(6,14), width=play*2)

		return Solid(
			coupling = coupling,
			magnets = magnets,
			screw = coupling_screw,
			)
	
	def build_hat():
		thickness = 2
		dscrew = stfloor(gearbox.shell.input.diameters[-1], 0.2)
		hat_bolt = bolt(
			gearbox.shell.input.perimeter.center + gearbox.shell.input.perimeter.radius*X,
			gearbox.shell.input.perimeter.center + gearbox.shell.input.perimeter.radius*X + dscrew*Z,
			dscrew)
		
		profile = wire([
			rotor_encoder.pose[3].xyz + dscrew*0.5*Z,
			rotor_encoder.pose[3].xyz + dscrew*0.5*(Z+X) + rotor_encoder.radius*1.1*X,
		
			hat_bolt.b + dscrew*1.5*X,
			hat_bolt.a + gearbox.shell.input.height*Z + dscrew*1.5*X,
			hat_bolt.a + gearbox.shell.input.height*Z - dscrew*X,
		
			hat_bolt.a*Z + gearbox.shell.input.height*Z + rotor_encoder.radius*1.1*X + dscrew*1.5*X,
			rotor_encoder.pose[3].xyz + rotor_encoder.radius*1.1*X,
			rotor_encoder.pose[3].xyz,	
			]).segmented()
		filet(profile, [1], width=dscrew)
		hat = revolution(profile)
		hat.mergeclose()
		
		screwing = circular_screwing(
			gearbox.shell.input.perimeter.axis,
			gearbox.shell.input.perimeter.radius,
			height = dscrew,
			dscrew = dscrew,
			div = 4,
			screws = True,
			)
		hat_body = intersection(hat, screwing.holes)
		
		holes = inflate(extrusion(web(rotor_encoder.holes), 3*thickness*Z, alignment=0.5), -0.2) .flip() .transform(rotor_encoder.pose)
		cables_way = extrusion(rotor_encoder.connections.group(1), gearbox.rext*Y) .transform(rotor_encoder.pose)
		cables = rotor_encoder.connections.box()
		support = gearbox.shell.input.perimeter.center + gearbox.shell.input.height*Z
		cables_way = parallelogram(
			(rotor_encoder.pose*cables.min)*Z - support, 
			cables.size.x * vec3(rotor_encoder.pose[0]), 
			gearbox.rext * vec3(rotor_encoder.pose[1]), 
			origin=support, 
			alignment=vec3(0.1, 0.5, 0),
			)
		#filet(cables_way, cables_way.frontiers(1,5,4), width=cables.size.z)
		cables_guide = inflate(cables_way.flip(), thickness)
		epsilon = 1e-1
		guide = intersection(cables_guide, inflate(expand(hat_body.group((6,5,4,3)), 10), -epsilon))
		hat_body = intersection(union(hat_body, guide), cables_way + holes)
		
		return Solid(
			body = hat_body.finish(),
			screws = screwing.screws,
			sensor = rotor_encoder,
			)
	
	
	def build_stator():
		bottom = motor.pose * (motor.length*-Z)
		radius = motor.interface.width/2
		overlap = motor.interface.width*0.01
		thickness = motor.interface.width*0.04
		offset = motor.interface.width*0.04
		epsilon = 1e-1
		overinterface = gearbox.output.perimeter.center - thickness*2*Z
		jaw = parallelogram(
			(motor.interface.width + offset*2)*X, 
			motor.interface.width*0.2*Y, 
			origin = bottom - epsilon*Z, 
			alignment = 0.5,
			fill = False,
			).flip()
		filet(jaw, jaw.frontiers(), width=2*offset)
		jaws = repeataround(extrusion(jaw, gearbox.output.perimeter.center - bottom + 2*epsilon), 3, angle=pi)
		body = revolution(wire([
			gearbox.output.perimeter.center + motor.interface.width/2*X + overlap*X,
			bottom + motor.interface.width/2*X - overlap*X,
			bottom + motor.interface.width/2*X + (thickness+offset)*X + (thickness+offset)*Z,
			
			overinterface + motor.interface.width/2*X + (thickness+offset)*X,
			overinterface + gearbox.output.perimeter.radius*X + gearbox.output.diameters[0]*1.5*X,
			gearbox.output.perimeter.center + gearbox.output.perimeter.radius*X + gearbox.output.diameters[0]*1.5*X,
			gearbox.output.perimeter.center + gearbox.output.perimeter.radius*X - gearbox.output.diameters[0]*X,
			]).close().segmented().flip())
		filet(body, body.frontiers(0,6), width=2*offset)
		screwing = circular_screwing(
			Axis(overinterface, Z), 
			gearbox.output.perimeter.radius,
			height = thickness*3,
			dscrew = gearbox.output.diameters[0],
	#		diameters = len(gearbox.output.diameters),
			expand = False,
			nuts = True,
			hold = thickness*2,
			)
		stator_coupling = intersection(body, jaws + screwing.holes)
	
		return Solid(body=stator_coupling.finish())
	
	return Solid(
		motor = motor,
		gearbox = gearbox,
		rotor = build_rotor(),
		hat = build_hat(),
		stator = build_stator(),
		)


if __name__ == '__madcad__':
#	settings.resolution = ('sqradm', 0.8)
	settings.resolution = ('sqradm', 0.3)
	
	j = joint()
	export(j, f"{__file__}/../out/joint-v2", (j.gearbox.rext, j.gearbox.height, round(j.motor.length)))
	