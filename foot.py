from madcad import *
from madcad.joints import *
from madcad.scheme import *



orange = vec3(0.8, 0.4, 0.1)*0.5

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
	foot_hole_big = stfloor(parallelogram_height*0.3, precision=0.3)
	foot_hole_small = stfloor(parallelogram_height*0.2, precision=0.3)
	nail_hole_big = stfloor(parallelogram_height*0.15, precision=0.3)
	nail_hole_small = stfloor(parallelogram_height*0.12, precision=0.3)
	front_hole = stfloor(mix(nail_hole_big, nail_hole_small, 0.5), precision=0.2)
	
	legdir = normalize(3*Z-X)
	OZ = Axis(O,Z)
	
	
	joints = [
		Revolute(('leg', 'traverse_front_bot'), foot_bot := Axis(0.5*parallelogram_height*Z, Y)),
		Revolute(('leg', 'traverse_front_top'), foot_top := Axis(1.5*parallelogram_height*Z, Y)),
		Revolute(('traverse_front_bot', 'front_dispatcher'), frontedge_bot := Axis(0.5*parallelogram_height*Z+front_dist*X, Y)),
		Revolute(('traverse_front_top', 'front_dispatcher'), frontedge_top := Axis(1.5*parallelogram_height*Z+front_dist*X, Y)),
		Revolute(('traverse_front_bot', 'backnail'), backnail_bot := Axis(0.5*parallelogram_height*Z-back_dist*X, Y)),
		Revolute(('traverse_front_top', 'backnail'), backnail_top := Axis(1.5*parallelogram_height*Z-back_dist*X, Y)),
	
		Revolute(('front_dispatcher', 'traverse_main_bot'), front_bot := Axis(vec3(front_dist, 0, 0), X)),
		Revolute(('front_dispatcher', 'traverse_main_top'), front_top := Axis(vec3(front_dist, 0, parallelogram_height), X)),
		Revolute(('traverse_main_bot', 'nail_front_right'), leftnail_bot := Axis(vec3(front_dist, 0.5*side_size, 0), X)),
		Revolute(('traverse_main_top', 'nail_front_right'), leftnail_top := Axis(vec3(front_dist, 0.5*side_size, parallelogram_height), X)),
		Revolute(('traverse_main_bot', 'nail_front_left'), rightnail_bot := Axis(vec3(front_dist, -0.5*side_size, 0), X)),
		Revolute(('traverse_main_top', 'nail_front_left'), rightnail_top := Axis(vec3(front_dist, -0.5*side_size, parallelogram_height), X)),
	
	#	PointSlider(('backnail', 'ground'), Axis(-3*X-2*Z, Z)),
	#	PointSlider(('nail_front_right', 'ground'), Axis(2*X+2*Y-2*Z, Z)),
	#	PointSlider(('nail_front_left', 'ground'), Axis(2*X-2*Y-2*Z, Z)),
	
	#	Ball(('backnail', 'ground'), -3*X-2*Z),
	#	Ball(('nail_front_right', 'ground'), 2*X+2*Y-2*Z),
	#	Ball(('nail_front_left', 'ground'), 2*X-2*Y-2*Z),
		]

	def nail_mount(r):
		d = 2*r
		l = stceil(2*(width+washer+r))
		return Solid(
			screw = standard.screw(d, l)
					.transform((width+washer+play)*Z),
			washera = standard.washer(d)
					.transform((0.5*width)*Z),
			washerb = standard.washer(d)
					.transform(-(0.5*width+washer)*Z),
			nut = standard.nut(d)
					.transform(-(width+washer+r)*Z),
			annotations = note_leading((width+washer+play+r)*Z, offset=d*(Z+X), text="M{:g}x{:g}".format(d, l)),
			)
	
	def frontnail_mount(r, a, b, c):
		d = 2*r
		l = stceil(a+b+c+2*washer+2*r)
		return Solid(
			screw = standard.screw(d, l, head='button')
					.transform((b+a+2*washer+2*play)*Z),
			washera = standard.washer(d)
					.transform((b+a+washer+2*play)*Z),
			washerb = standard.washer(d)
					.transform((b)*Z),
			nut = standard.nut(d)
					.transform((-c-r)*Z),
			annotations = note_leading((b+a+2*washer+2*play+r)*Z, offset=d*(Z+X), text="M{:g}x{:g}".format(d, l)),
			)
	
	holes = mesh.mesh([
			centercylinder(foot_top, foot_hole_big, 5*width),
			centercylinder(foot_bot, foot_hole_small, 5*width),
			centerslot(backnail_top, nail_hole_big, 2*(width+washer+play)),
			centerslot(backnail_bot, nail_hole_small, 2*(width+washer+play)),
			centerslot(frontedge_top, nail_hole_big, 2*(width+washer+play)),
			centerslot(frontedge_bot, nail_hole_small, 2*(width+washer+play)),
			])
	
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
	
	leg_profile = convexoutline(web([
		Circle(foot_top.transform(front_size*Z), 2*foot_hole_big),
		Circle(foot_top, 2*foot_hole_big),
		Circle(foot_bot, 2*foot_hole_small),
		]))


	def leg():
		thickness = 1.5
		body = difference(
			extrusion(
				flatsurface(leg_profile),
				width*Y,
				alignment=0.5,
				).orient(),
			holes)
		return Solid(
			body = body,
			bearing_top = slidebearing(2*foot_hole_big-2*thickness, 2.5*width, thickness) 
				.place((Revolute, OZ, foot_top)) .transform(1.25*width*Y),
			bearing_bot = slidebearing(2*foot_hole_small-2*thickness, 2.5*width, thickness) 
				.place((Revolute, OZ, foot_bot)) .transform(1.25*width*Y),
			)
	
	def backnail():
		tiphigh = revolution(Softened([
			vec3(0, 0., -nail_height),
			vec3(-0.25*nail_width, 0., -nail_height),
			vec3(-0.5*nail_width, 0., -nail_height*0.5),
			vec3(-0.5*nail_width, 0., -nail_height*0.3),
			vec3(-0.3*nail_width, 0., 1.9*parallelogram_height),
			vec3(0, 0., 1.9*parallelogram_height),
			])).flip()
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
		backnail_interference_profile = leg_profile.transform(rotatearound(front_angle, backnail_top) * foot_top.origin - foot_top.origin)
		play = foot_hole_big*0.1
		backnail = difference(backnail, inflate(extrusion(backnail_interference_profile, nail_width*Y, alignment=0.5).orient(), play))
		backnail = difference(backnail, inflate(holes, play))
		return backnail.finish()
	
	def nails_front_():
		tiplow = revolution(Softened([
			vec3(0, 0., -nail_height),
			vec3(-0.25*nail_width, 0., -nail_height),
			vec3(-0.5*nail_width, 0., -nail_height*0.5),
			vec3(-0.5*nail_width, 0., -nail_height*0.3),
			vec3(-0.25*nail_width, 0., 1.35*parallelogram_height),
			vec3(0, 0., 1.35*parallelogram_height),
			])).flip()
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

		return Solid(
			right = difference(frontnail, front_holes).finish(),
			left = difference(frontnail.transform(scaledir(Y,-1)).flip(), front_holes).finish(),
			)
	
	def traverses_front_():
		bot_profile = convexoutline(web([
			Circle(foot_bot.transform(-0.1*parallelogram_height*Z), 1.8*foot_hole_small) .mesh().transform(scaledir(X,1.5)),
			Circle(backnail_bot, 2*nail_hole_small),
			Circle(Axis(0.5*parallelogram_height*Z+front_dist*X,Y), 2*nail_hole_small),
			]))
		top_profile = convexoutline(web([
			Circle(foot_top.transform(0.1*parallelogram_height*Z), 1.8*foot_hole_big) .mesh().transform(scaledir(X,1.5)),
			Circle(backnail_top, 2*nail_hole_big),
			Circle(Axis(1.5*parallelogram_height*Z+front_dist*X,Y), 2*nail_hole_big),
			]))
	
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
		bot_body = difference(
			intersection(
				intersection(
					extrusion(edgeside, front_size*3*X, alignment=0.5).orient(),
					extrusion(bot_profile, 4*width*Y, alignment=0.5).orient(),
					),
				extrusion(edgetop, front_size*3*Z).orient(),
				),
			brick(center=O, width=(width+2*(washer+play))*Y + 4*front_size*(X+Z)),
			)
		top_body = difference(intersection(
				intersection(
					extrusion(edgeside, front_size*3*X, alignment=0.5).orient(),
					extrusion(top_profile, 4*width*Y, alignment=0.5).orient(),
					),
				extrusion(edgetop, front_size*3*Z).orient(),
				),
			brick(center=O, width=(width+2*(washer+play))*Y + 4*front_size*(X+Z)),
			)

		nail_mount_small = nail_mount(nail_hole_small)
		nail_mount_big = nail_mount(nail_hole_big)

		return Solid(
			bot = Solid(
				body = difference(bot_body, holes) .finish().option(color=orange),
				bolt_back = nail_mount_small.place((Revolute, OZ, backnail_bot)),
				bolt_front = nail_mount_small.place((Revolute, OZ, frontedge_bot)),
				),
			top = Solid(
				body = difference(top_body, holes) .finish().option(color=orange),
				bolt_back = nail_mount_big.place((Revolute, OZ, backnail_top)),
				bolt_front = nail_mount_big.place((Revolute, OZ, frontedge_top)),
				),
			)
	
	
	def traverses_main_():
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
		
		bot_profile = convexoutline(web([
			Circle(Axis(front_dist*X, X), 1*width),
			Circle(Axis(front_dist*X + side_dist*Y, X), 2.2*nail_hole_small),
			Circle(Axis(front_dist*X - side_dist*Y, X), 2.2*nail_hole_small),
			]))
		bot = intersection(
			extrusion(bot_profile.transform(0.5*width*X), 
				4*width*X, 
				alignment=0.1).orient(),
			frontedge_top_profile,
			)
		top_profile = convexoutline(web([
			Circle(Axis(parallelogram_height*Z + front_dist*X, X), 1.2*width),
			Circle(Axis(parallelogram_height*Z + front_dist*X + side_dist*Y, X), 2.2*nail_hole_small),
			Circle(Axis(parallelogram_height*Z + front_dist*X - side_dist*Y, X), 2.2*nail_hole_small),
			]))
		top = intersection(
			extrusion(top_profile.transform(0.5*width*X), 
				2*width*X + 2*front_hole*X, 
				alignment=0.1).orient(),
			frontedge_top_profile.transform(parallelogram_height*Z),
			)
	
		frontnail_mount_small = frontnail_mount(nail_hole_small, width, width/2, width/2)
		
		return Solid(
			bot = Solid(
				body = difference(bot, inflate(front_holes, play)) .finish().option(color=orange),
				left = frontnail_mount_small.place((Revolute, OZ, leftnail_bot)),
				right = frontnail_mount_small.place((Revolute, OZ, rightnail_bot)),
				),
			top = Solid(
				body = difference(top, inflate(front_holes, play)) .finish().option(color=orange),
				left = frontnail_mount_small.place((Revolute, OZ, leftnail_top)),
				right = frontnail_mount_small.place((Revolute, OZ, rightnail_top)),
				),
			)

	def front_dispatcher():
		front_dispatcher = convexhull(
			cylinder(
				+0.5*width*Y + 1.5*parallelogram_height*Z + front_dist*X,
				-0.5*width*Y + 1.5*parallelogram_height*Z + front_dist*X,
				2.2*nail_hole_big)
			+ cylinder(
				+2.2*nail_hole_big*X + front_dist*X,
				-2.2*nail_hole_big*X + front_dist*X,
				0.5*width)
			)
		frontnail_mount_big = frontnail_mount(front_hole, width, 2.2*nail_hole_big, 2.2*nail_hole_big-2.5*front_hole)
		return Solid(
			body = difference(front_dispatcher, inflate(holes, play) + front_holes).finish(),
			bolt_top = frontnail_mount_big.place((Revolute, OZ, front_top)),
			bolt_bot = frontnail_mount_big.place((Revolute, OZ, front_bot)),
			)
	
	
	nails_front = nails_front_()
	traverses_main = traverses_main_()
	traverses_front = traverses_front_()
	kin = Kinematic(joints, ground='leg', content={
		'leg': leg(),
		'backnail': backnail(),
		'nail_front_right': nails_front.right,
		'nail_front_left': nails_front.left,
	
		'front_dispatcher': front_dispatcher(),
		'traverse_front_bot': traverses_front.bot,
		'traverse_front_top': traverses_front.top,
		'traverse_main_bot': traverses_main.bot,
		'traverse_main_top': traverses_main.top,
		})
	return Solid(
		kinematic = kin,
		axis = Axis(-nail_height*Z, Z),
		)


if __name__ == '__madcad__':		
	settings.resolution = ('sqradm', 0.8)

#	foot1 = foot(50, 45, 20)
#	foot2 = foot(50*2, 45*2, 20*2)
#	foot2 = foot(120, 110, 50)
	foot3 = foot(60, 55, 25)


# TODO try non parallel (evasive) nails to reduce bulkiness and improve stability
