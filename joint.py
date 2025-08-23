from madcad import *
from gearbox.strainwave import strainwave_dual_crown
import nema

gearbox = strainwave_dual_crown(
		rext = 50,
		nteeth = 60,
		guided = True,
		)
motor = nema.motor(17, 51, 5, 20, round=True, coupling='flat')
#motor = nema.motor(17, 43, coupling='flat').transform(translate(20*Z) * rotate(pi,X))

bottom = motor.length*-Z
r = motor.interface.width/2
c = Circle(Axis(bottom, Z), r*0.98)
t = 0.1*r
p = parallelogram(
	r*2*1.05*X, 
	r*2*0.3*Y, 
	origin=bottom, 
	align=0.5, 
	fill=False)
e = wire([
	bottom + r*X,
	bottom + r*X + t*X,
	gearbox.output.int.perimeter + gearbox.output.int.perimeter.radius*X.
	])