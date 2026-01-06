from madcad import *
from madcad.joints import *


@cachefunc
def foot(
		# overall foot dimensions
		front_size,
		side_size,
		# structure dimensions
		parallelogram_height,   # distance between joints in the parallelogram
		# foot capabilities
		front_angle = radians(50),
		side_angle = radians(35),
		# assembly settings
		play = 0.2,
		):

	front_dist = 2/5*front_size
	back_dist = 3/5*front_size
	side_dist = 0.5*side_size
	width = 0.4 * parallelogram_height
	washer = stceil(width*0.1)
	
	# overall nail dimensions
	nail_width = parallelogram_height * 1.5
	nail_height = parallelogram_height * 1.6
	# hole sizes
	foot_hole_big = stceil(parallelogram_height*0.25, precision=0.2)
	foot_hole_small = stceil(parallelogram_height*0.18, precision=0.2)
	nail_hole_big = stceil(parallelogram_height*0.15, precision=0.5)
	nail_hole_small = stceil(parallelogram_height*0.1, precision=0.5)
	front_hole = stceil(mix(nail_hole_big, nail_hole_small, 0.5), precision=0.5)
	
	legdir = normalize(3*Z-X)
	
	orange = vec3(0.8, 0.4, 0.1)*0.5
	
	leg_bot = Axis(30*Z+80*legdir, Y)
	leg_top = Axis(30*Z+120*legdir, Y)
	joints = [
#		Revolute(('leg', 'foreedge'), leg_bot := Axis(30*Z+80*legdir, Y)),
#		Revolute(('leg', 'backedge'), leg_top := Axis(30*Z+120*legdir, Y)),
	
		Revolute(('leg', 'traverse0'), foot_bot := Axis(0.5*parallelogram_height*Z, Y)),
		Revolute(('leg', 'traverse1'), foot_top := Axis(1.5*parallelogram_height*Z, Y)),
		Revolute(('traverse0', 'side0'), frontedge_bot := Axis(0.5*parallelogram_height*Z+front_dist*X, Y)),
		Revolute(('traverse1', 'side0'), frontedge_top := Axis(1.5*parallelogram_height*Z+front_dist*X, Y)),
		Revolute(('traverse0', 'side1'), backnail_bot := Axis(0.5*parallelogram_height*Z-back_dist*X, Y)),
		Revolute(('traverse1', 'side1'), backnail_top := Axis(1.5*parallelogram_height*Z-back_dist*X, Y)),
	
		Revolute(('side0', 'fronttraverse0'), front_bot := Axis(vec3(front_dist, 0, 0), X)),
		Revolute(('side0', 'fronttraverse1'), front_top := Axis(vec3(front_dist, 0, parallelogram_height), X)),
		Revolute(('fronttraverse0', 'nail0'), leftnail_bot := Axis(vec3(front_dist, 0.5*side_size, 0), X)),
		Revolute(('fronttraverse1', 'nail0'), leftnail_top := Axis(vec3(front_dist, 0.5*side_size, parallelogram_height), X)),
		Revolute(('fronttraverse0', 'nail1'), rightnail_bot := Axis(vec3(front_dist, -0.5*side_size, 0), X)),
		Revolute(('fronttraverse1', 'nail1'), rightnail_top := Axis(vec3(front_dist, -0.5*side_size, parallelogram_height), X)),
	
	#	PointSlider(('side1', 'ground'), Axis(-3*X-2*Z, Z)),
	#	PointSlider(('nail0', 'ground'), Axis(2*X+2*Y-2*Z, Z)),
	#	PointSlider(('nail1', 'ground'), Axis(2*X-2*Y-2*Z, Z)),
	
	#	Ball(('side1', 'ground'), -3*X-2*Z),
	#	Ball(('nail0', 'ground'), 2*X+2*Y-2*Z),
	#	Ball(('nail1', 'ground'), 2*X-2*Y-2*Z),
		]
	
	tiphigh = revolution(Softened([
		vec3(0, 0., -nail_height),
		vec3(-0.25*nail_width, 0., -nail_height),
		vec3(-0.5*nail_width, 0., -nail_height*0.5),
		vec3(-0.5*nail_width, 0., -nail_height*0.3),
		vec3(-0.3*nail_width, 0., 1.9*parallelogram_height),
		vec3(0, 0., 1.9*parallelogram_height),
		])).flip()
	tiplow = revolution(Softened([
		vec3(0, 0., -nail_height),
		vec3(-0.25*nail_width, 0., -nail_height),
		vec3(-0.5*nail_width, 0., -nail_height*0.5),
		vec3(-0.5*nail_width, 0., -nail_height*0.3),
		vec3(-0.25*nail_width, 0., 1.35*parallelogram_height),
		vec3(0, 0., 1.35*parallelogram_height),
		])).flip()
	
	leg_attach = convexoutline(web([
		Circle(foot_top.transform(front_size*Z), 2*foot_hole_big),
		Circle(foot_top, 2*foot_hole_big),
		Circle(foot_bot, 2*foot_hole_small),
		]))
	
	backnail_attach_profile = convexoutline(web([
		Circle(backnail_top, 2.4*nail_hole_big),
		Circle(backnail_bot, 2.4*nail_hole_small),
		Circle(
			foot_bot.transform(rotatearound(front_angle, backnail_bot)), 
			2.3*foot_hole_small),
		])).orient().finish()

	backnail = difference(tiphigh.transform(project(backnail_bot.origin, X)),
		extrusion(
			flatsurface(backnail_attach_profile).transform(0.5*width*Y).flip(), 
			nail_width*Y).orient()
		+ extrusion(
			flatsurface(backnail_attach_profile).transform(-0.5*width*Y), 
			-nail_width*Y).orient()
		)
	backnail_interference_profile = leg_attach.transform(rotatearound(front_angle, backnail_top) * foot_top.origin - foot_top.origin)
	play = foot_hole_big*0.1
	backnail = difference(backnail, inflate(extrusion(backnail_interference_profile, nail_width*Y, alignment=0.5).orient(), play))
	
	
	frontnail_attach_profile = convexoutline(web([
		Circle(leftnail_top, 2.6*nail_hole_small),
		Circle(leftnail_bot, 2.4*nail_hole_small),
		Circle(front_top.transform(rotatearound(side_angle, leftnail_top)), 1.6*front_hole),
		Circle(front_bot.transform(rotatearound(side_angle, leftnail_bot)), 1.6*front_hole),
		]))
	frontnail = difference(tiplow.transform(leftnail_bot.origin*vec3(1,1,0)),
		extrusion(
			flatsurface(frontnail_attach_profile).transform(0.5*width*X).flip(), 
			2*width*X).orient()
		+ extrusion(
			flatsurface(frontnail_attach_profile).transform(-0.5*width*X), 
			-2*width*X).orient()
		)
	frontnail_interference_profile = convexoutline(web([
		Circle(front_top.transform(rotatearound(-side_angle, leftnail_top)), 1.7*width),
		Circle(front_bot.transform(rotatearound(side_angle, leftnail_bot)).transform(0.5*parallelogram_height*Z), 1.7*width),
		Circle(front_bot.transform(rotatearound(side_angle, leftnail_bot)), 2*front_hole),
		]))
	frontnail = difference(frontnail, extrusion(frontnail_interference_profile, 2*nail_width*X, alignment=0.5).orient())
	
	traverse0_profile = convexoutline(web([
		Circle(foot_bot.transform(-0.1*parallelogram_height*Z), 1.8*foot_hole_small) .mesh().transform(scaledir(X,1.5)),
		Circle(backnail_bot, 2*nail_hole_small),
		Circle(Axis(0.5*parallelogram_height*Z+front_dist*X,Y), 2*nail_hole_small),
		]))
	traverse0 = mesh.mesh([
		extrusion(flatsurface(traverse0_profile).transform(+(0.5*width+washer+play)*Y), 0.5*width*Y, alignment=0).orient(),
		extrusion(flatsurface(traverse0_profile).transform(-(0.5*width+washer+play)*Y), 0.5*width*Y, alignment=1).orient(),
		])
	traverse1_profile = convexoutline(web([
		Circle(foot_top.transform(0.1*parallelogram_height*Z), 1.8*foot_hole_big) .mesh().transform(scaledir(X,1.5)),
		Circle(backnail_top, 2*nail_hole_big),
		Circle(Axis(1.5*parallelogram_height*Z+front_dist*X,Y), 2*nail_hole_big),
		]))
	traverse1 = mesh.mesh([
		extrusion(flatsurface(traverse1_profile).transform((0.5*width+washer+play)*Y), 0.5*width*Y, alignment=0).orient(),
		extrusion(flatsurface(traverse1_profile).transform(-(0.5*width+washer+play)*Y), 0.5*width*Y, alignment=1).orient(),
		])
	
	# BUG: convexoutline returns overlapping edges
	edgeside = convexoutline(web([
		Circle(Axis(foot_top.origin + width*Z, X), width),
		Circle(Axis(foot_top.origin - foot_hole_big*Z, X), 1.8*width),
		Circle(Axis(foot_bot.origin + foot_hole_small*Z, X), 1.8*width),
		Circle(Axis(foot_bot.origin - width*Z, X), width),
		])).mergegroups()
	edgetop = convexoutline(web([
		Circle(Axis(O,Z), 1.8*width).mesh() .transform(scaledir(X, 2)),
		Circle(Axis(backnail_bot.origin*(1.1*X+Y), Z), 1.2*width),
		Circle(Axis(frontedge_bot.origin*(1.1*X+Y), Z), 1.2*width),
		])).mergegroups()
	traverse0 = difference(
		intersection(
			intersection(
				extrusion(edgeside, front_size*3*X, alignment=0.5).orient(),
				extrusion(traverse0_profile, 4*width*Y, alignment=0.5).orient(),
				),
			extrusion(edgetop, front_size*3*Z).orient(),
			),
		brick(center=O, width=(width+2*(washer+play))*Y + 4*front_size*(X+Z)),
		)
	traverse1 = difference(intersection(
			intersection(
				extrusion(edgeside, front_size*3*X, alignment=0.5).orient(),
				extrusion(traverse1_profile, 4*width*Y, alignment=0.5).orient(),
				),
			extrusion(edgetop, front_size*3*Z).orient(),
			),
		brick(center=O, width=(width+2*(washer+play))*Y + 4*front_size*(X+Z)),
		)
	
	side0 = convexhull(
		cylinder(
			+0.5*width*Y + 1.5*parallelogram_height*Z + front_dist*X,
			-0.5*width*Y + 1.5*parallelogram_height*Z + front_dist*X,
			2.2*nail_hole_big)
		+ cylinder(
			+2.2*nail_hole_big*X + front_dist*X,
			-2.2*nail_hole_big*X + front_dist*X,
			0.5*width)
		)
	
	#bolt(-back_dist*X - 1.5*width*Y + 0.5*parallelogram_height, 
	
	frontedge_top_profile = revolution(web([
		wire([
			leftnail_bot.origin + 3*nail_hole_small*Y + 0.5*width*X + (washer+play)*X,
			leftnail_bot.origin - 1.5*nail_hole_small*Y + 0.5*width*X + (washer+play)*X,
			front_bot.origin + (washer+play)*X + 2*width*Y + 2.2*nail_hole_big*X,
			front_bot.origin + (washer+play)*X + 2.2*nail_hole_big*X,
			]).segmented(),
		wire([
			front_bot.origin + 2.2*nail_hole_big*X + width*X + washer*X + front_hole*X,
			front_bot.origin + 2.2*nail_hole_big*X + width*X + washer*X + front_hole*X + 2.5*front_hole*Y,
			leftnail_bot.origin + 1.5*width*X + 3*nail_hole_small*Y,
			]),
		]).segmented(), Axis(O,X))
	frontedge_top_profile.mergeclose()
	
	fronttraverse0_profile = convexoutline(web([
		Circle(Axis(front_dist*X, X), 1*width),
		Circle(Axis(front_dist*X + side_dist*Y, X), 2.2*nail_hole_small),
		Circle(Axis(front_dist*X - side_dist*Y, X), 2.2*nail_hole_small),
		]))
	fronttraverse0 = intersection(
			extrusion(fronttraverse0_profile.transform(0.5*width*X), 
				2*width*X, 
				alignment=0.1).orient(),
			frontedge_top_profile,
			)
	fronttraverse1_profile = convexoutline(web([
		Circle(Axis(parallelogram_height*Z + front_dist*X, X), 1.2*width),
		Circle(Axis(parallelogram_height*Z + front_dist*X + side_dist*Y, X), 2.2*nail_hole_small),
		Circle(Axis(parallelogram_height*Z + front_dist*X - side_dist*Y, X), 2.2*nail_hole_small),
		]))
	fronttraverse1 = intersection(
			extrusion(fronttraverse1_profile.transform(0.5*width*X), 
				2*width*X, 
				alignment=0.1).orient(),
			frontedge_top_profile.transform(parallelogram_height*Z),
			)
	
	def centercylinder(axis, radius, height):
		return cylinder(
			axis.origin - axis.direction*0.5*height, 
			axis.origin + axis.direction*0.5*height,
			radius)
	def centerslot(axis, radius, height):
		return bolt_slot(
			axis.origin - axis.direction*0.5*height, 
			axis.origin + axis.direction*0.5*height,
			dscrew = 2*radius,
			rslot = 2.2*radius).flip()
	def centerbolt(axis, radius, height):
		return bolt(
			axis.origin - axis.direction*0.5*height, 
			axis.origin + axis.direction*0.5*height,
			radius)
	
	OZ = Axis(O,Z)
	def nail_mount(r):
		d = 2*r
		return Solid(
			screw = standard.screw(d, stceil(2*(width+washer+r)))
					.transform((width+washer+play)*Z),
			washera = standard.washer(d)
					.transform((0.5*width)*Z),
			washerb = standard.washer(d)
					.transform(-(0.5*width+washer)*Z),
			nut = standard.nut(d)
					.transform(-(width+washer+r)*Z),
			)
	nail_mount_small = nail_mount(nail_hole_small)
	nail_mount_big = nail_mount(nail_hole_big)
	
	holes = mesh.mesh([
			centercylinder(foot_top, foot_hole_big, 5*width),
			centercylinder(foot_bot, foot_hole_small, 5*width),
			centerslot(backnail_top, nail_hole_big, 2*(width+washer+play)),
			centerslot(backnail_bot, nail_hole_small, 2*(width+washer+play)),
			centerslot(frontedge_top, nail_hole_big, 2*(width+washer+play)),
			centerslot(frontedge_bot, nail_hole_small, 2*(width+washer+play)),
			])
	traverse0 = Solid(
		body = difference(traverse0, holes) .finish().option(color=orange),
		bolt_back = nail_mount_small.place((Revolute, OZ, backnail_bot)),
		bolt_front = nail_mount_small.place((Revolute, OZ, frontedge_bot)),
		)
	traverse1 = Solid(
		body = difference(traverse1, holes) .finish().option(color=orange),
		bolt_back = nail_mount_big.place((Revolute, OZ, backnail_top)),
		bolt_front = nail_mount_big.place((Revolute, OZ, frontedge_top)),
		)
	backnail = difference(backnail, inflate(holes, play))
	
	front_holes = mesh.mesh([
			bolt_slot(
				front_top.offset(-2.2*nail_hole_big + 2.5*front_hole).origin,  
				front_top.offset(2.2*nail_hole_big + washer + play + width).origin, 
				2*front_hole,
				2.2*front_hole).flip(),
			bolt_slot(
				front_bot.offset(-2.2*nail_hole_big + 2.5*front_hole).origin,  
				front_bot.offset(2.2*nail_hole_big + washer + play + width).origin, 
				2*front_hole,
				2.2*front_hole).flip(),
			centerslot(leftnail_top, nail_hole_small, 3*width + 2*(washer+play)*X),
			centerslot(leftnail_bot, nail_hole_small, 3*width + 2*(washer+play)*X),
			centerslot(rightnail_top, nail_hole_small, 3*width + 2*(washer+play)*X),
			centerslot(rightnail_bot, nail_hole_small, 3*width + 2*(washer+play)*X),
		])
	def frontnail_mount(r, a, b, c):
		d = 2*r
		return Solid(
			screw = standard.screw(d, stceil(a+b+c+2*washer+2*r), head='button')
					.transform((b+a+2*washer+2*play)*Z),
			washera = standard.washer(d)
					.transform((b+a+washer+2*play)*Z),
			washerb = standard.washer(d)
					.transform((b)*Z),
			nut = standard.nut(d)
					.transform((-c-r)*Z),
			)
	frontnail_mount_small = frontnail_mount(nail_hole_small, width, width/2, width/2)
	frontnail_mount_big = frontnail_mount(front_hole, width, 2.2*nail_hole_big, 2.2*nail_hole_big-2.5*front_hole)
	
	rightnail = difference(frontnail, front_holes)
	leftnail = difference(frontnail.transform(scaledir(Y,-1)).flip(), front_holes)
	fronttraverse0 = [
		difference(fronttraverse0, inflate(front_holes, play)) .finish().option(color=orange),
		frontnail_mount_small.place((Revolute, OZ, leftnail_bot)),
		frontnail_mount_small.place((Revolute, OZ, rightnail_bot)),
		]
	fronttraverse1 = [
		difference(fronttraverse1, inflate(front_holes, play)) .finish().option(color=orange),
		frontnail_mount_small.place((Revolute, OZ, leftnail_top)),
		frontnail_mount_small.place((Revolute, OZ, rightnail_top)),
		]
	side0 = [
		difference(side0, inflate(holes, play) + front_holes),
		frontnail_mount_big.place((Revolute, OZ, front_top)),
		frontnail_mount_big.place((Revolute, OZ, front_bot)),
		]
	
	thickness = 1.5
	foot = [
		slidebearing(2*foot_hole_big-2*thickness, 2.5*width, thickness) 
			.place((Revolute, OZ, foot_top)) .transform(1.25*width*Y),
		slidebearing(2*foot_hole_small-2*thickness, 2.5*width, thickness) 
			.place((Revolute, OZ, foot_bot)) .transform(1.25*width*Y),
		]
	
	kin = Kinematic(joints, ground='leg', content={
		'foot': foot,
		'side1': backnail,
		'nail0': rightnail,
		'nail1': leftnail,
	
		'side0': side0,
		'traverse0': traverse0,
		'traverse1': traverse1,
		'fronttraverse0': fronttraverse0,
		'fronttraverse1': fronttraverse1,
		})
	return Solid(
		kinematic = kin,
		axis = Axis(-nail_height*Z, Z),
		attach = leg_attach,
		)


if __name__ == '__madcad__':		
	settings.resolution = ('sqradm', 0.8)

#	foot1 = foot(50, 45, 20)
	foot2 = foot(50*2, 45*2, 20*2)
