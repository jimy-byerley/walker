from madcad import *
from madcad.gear import gearprofile, rackprofile
from madcad.joints import *
import numpy as np
import scipy.optimize
from functools import reduce
import operator

#settings.resolution = ('rad', 0.2)
settings.resolution = ('sqradm', 0.2)
#settings.resolution = ('sqradm', 0.8)



def perimeter(f, start, stop, div=128):
	points = [f(x)  for x in linrange(start, stop, div=div)]
	return sum(distance(points[i-1], points[i])  for i in range(1, len(points)))

def meshing(e):
	# ellipsis
	# def flex(a, b, phase):
		# return vec2(a*cos(phase), b*sin(phase))
	# def dflex(a, b, phase):
	# 	return vec2(-a*sin(phase), b*cos(phase))
	
	# offseted ellipsis
	# def flex1(a, b, phase):
	# 	return vec2(a*cos(phase), b*sin(phase))
	# def dflex1(a, b, phase):
	# 	return vec2(-a*sin(phase), b*cos(phase))
	# def flex(a, b, phase):
	# 	return flex1(a-e, b-e, phase) - perp(normalize(dflex1(a-e, b-e, phase))) * e
	# def dflex(a, b, phase):
	# 	dx = 1e-6
	# 	return (flex(a,b,phase+dx) - flex(a,b,phase-dx)) / (2*dx)
	
	# sine deformation wave
	def flex(a, b, phase):
		return vec2(cos(phase), sin(phase)) * mix(b, a, linstep(-1, 1, cos(2*phase)))
	def dflex(a, b, phase):
		dx = 1e-6
		return (flex(a,b,phase+dx) - flex(a,b,phase-dx)) / (2*dx)

	# ramps
	#def flex(a, b, phase):
	#	quater = phase % pi
	#	if quater > pi/2:	quater = pi-quater
	#	return vec2(cos(phase), sin(phase)) * mix(a, b, quater*2/pi)
	#def dflex(a, b, phase):
	#	dx = 1e-6
	#	return (flex(a, b, phase) - flex(a, b, phase-dx))/dx

	maxradius = 40
	circle_teeth = 60
	flex_teeth = circle_teeth-2

	circle_radius = maxradius
	flex_max = maxradius
	def predicate(flex_min):
		circle_perimeter = 2*pi*circle_radius
		flex_perimeter = perimeter(lambda phase: flex(flex_max, flex_min, phase), 0, 2*pi)
		return flex_perimeter > circle_perimeter*flex_teeth/circle_teeth
	flex_min = fbisect(predicate, maxradius, 0)
	print('flex', flex_min, flex_max)
	
	# flex_min = 32

	def cost(phases):
		points = np.array([
			distance(
				flex(flex_max, flex_min, phases[i-1]), 
				flex(flex_max, flex_min, phases[i]))
			for i in range(len(phases))
			])
		return points - np.mean(points)
	phases = np.linspace(0, 2*pi, flex_teeth, endpoint=False)
	result = scipy.optimize.least_squares(cost, phases)
	assert result.success
	phases = result.x
	phases -= phases[0]

	base = wire(vec3(flex(flex_max, flex_min, phase),0)  for phase in phases).close()

	teeth_step = 2*pi*circle_radius / (circle_teeth-3)
	teeth_height = 0.3*teeth_step
	#tooth = (
	#	rackprofile(teeth_step, height=teeth_step*0.25, asymetry=teeth_height*0.05, pressure_angle=radians(30))
	#	.transform(translate(teeth_step*0.5 * Y - teeth_step*0.5*X) * scale(vec3(1,-1,1)))
	#	.flip()
	#	)
	tooth = (
		gearprofile(teeth_step, circle_teeth, 
			height=teeth_height, 
			asymetry=-teeth_height*0.2, 
	#		offset=-teeth_height*0.2,
			pressure_angle=radians(30),
			)
		.transform(-base[0])
		)
	tooth.indices.pop()
	tooth.tracks.pop()
	flex_profile = wire([tooth.transform(
		translate(vec3(flex(flex_max, flex_min, phase), 0))
		* mat4(quat(Y, vec3(normalize(dflex(flex_max, flex_min, phase)), 0)))
		) for phase in phases])

	# teeth
	circle_profile = repeat(
		gearprofile(teeth_step, circle_teeth, 
			height=teeth_height, 
			asymetry=teeth_height*0.2, 
			pressure_angle=radians(30),
			),
		circle_teeth, 
		rotate(2*pi/circle_teeth, Z),
		).option(color=vec3(0,0.5,1))
	out_profile = repeat(
		gearprofile(teeth_step, circle_teeth, 
			height=teeth_height, 
	#		asymetry=teeth_height*0.2, 
			offset=teeth_height*0.2,
			pressure_angle=radians(33),
			),
		flex_teeth, 
		rotate(2*pi/flex_teeth, Z),
		).option(color=vec3(0,1,0.5))
		
	return [
		base.option(color=vec3(1-e/maxradius, 1,0)), 
		flex_profile.option(color=vec3(1-e/maxradius, 1,0)), 
		circle_profile, out_profile]


show([
	meshing(0), 
	# meshing(34),
	])
