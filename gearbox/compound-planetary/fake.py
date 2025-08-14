from madcad import *

settings.primitives['curve_resolution'] = ('sqradm', 0.8)

bearing = dict(dint=70, dext=90, h=10)
dscrew = 6
diameters = 2
div = 8

axis = Axis(O,Z)

shell_screwing_radius = stceil(bearing['dext']/2 + dscrew*0.75)
screw_height = 1.4*bearing['h']
bearing_placeholder = standard.bearing(**bearing)

shell_in_radius = mix(bearing['dint'], bearing['dext'], 0.7)/2
holes, bolts = standard.circular_screwing(
	axis.offset(-screw_height/2), 
	shell_screwing_radius, 
	screw_height,
	dscrew,
	diameters,
	div,
	)
profile = wire([
	shell_screwing_radius*X + dscrew*X - screw_height*0.5*Z,
	shell_in_radius*X - screw_height*0.5*Z,
	shell_in_radius*X - bearing['h']*0.5*Z,
	bearing['dext']*0.5*X - bearing['h']*0.5*Z,
	])
shell = revolution((
	profile + 
	profile.transform(scaledir(Z,-1)).flip()
	).close().segmented())
shell = intersection(shell, holes)

hub_bot = 0.5*screw_height
hub_top = 0.5*screw_height
hub_screwing_radius = stfloor(bearing['dint']/2 - dscrew*0.75)
holes, bolts = standard.circular_screwing(
	axis.offset(-hub_bot), 
	hub_screwing_radius, 
	hub_bot + hub_top,
	dscrew,
	diameters,
	div,
	)

hub_out_radius = mix(bearing['dint'], bearing['dext'], 0.3)/2
profile = wire([
	bearing['dint']*0.5*X - bearing['h']*0.5*Z,
	bearing['dint']*0.5*X + bearing['h']*0.5*Z,	
	hub_out_radius*X + bearing['h']*0.5*Z,
	hub_out_radius*X + hub_top*Z,
	hub_screwing_radius*X - dscrew*X + hub_top*Z,
	hub_screwing_radius*X - dscrew*X - bearing['h']*0.5*Z,
	]).close().segmented().flip()
hub = revolution(profile)