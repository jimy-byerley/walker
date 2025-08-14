def compound_planetary(
	zcrown_out = 51
	zcrown_in = 44 + 2*9
	zplanet_in = 9
	zplanet_out = 8
	rext = 50
	play = 0.1
	bearing = None
	):
	
	# global parameters
	height = stceil(0.2*rext)
	dscrew_out = stceil(rext*0.1)
	dscrew_in = stceil(rext*0.08)
	shell_thickness = stceil(0.03 * rext)
	nplanets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
	zsun = zcrown_in - 2*zplanet_in
	
	axis = Axis(O,Z)
	
	tmp = Solid()
	guides(tmp)
	kinematic(tmp)
	teeth(tmp)
	
def kinematic(guides):
	joints = []
	content = {
		'crown_out': [out_receiver, out_int, bolts_out_int],
		'sun': input,
		}
	for i, phase in enumerate(guides.phases):
		p = rotate(phase,Z) * guides.planet_in.center
		joints.extend([
			Gear(('sun', 'planet_{}'.format(i)), -zsun/guides.zplanet_in, p, axis, Axis(p,Z)),
			Gear(('crown_in', 'planet_{}'.format(i)), guides.zcrown_in/guides.zplanet_in, p, axis, Axis(p,Z)),
			Gear(('crown_out', 'planet_{}'.format(i)), guides.zcrown_out/guides.zplanet_out, p, axis, Axis(p,Z)),
			])
		content['planet_{}'.format(i)] = planets[i]
	joints.append(Revolute(('sun', 'crown_in'), axis))
	return Kinematic(joints, ground='crown_in', content=content)
	
def kinematic(self):
	joints = []
	for i, phase in enumerate(self.planets_phases):
		p = rotate(phase,Z) * self.planet_in.center
		joints.extend([
			Gear(('sun', 'planet_{}'.format(i)), -self.zsun/self.zplanet_in, p, self.axis, Axis(p,Z)),
			Gear(('crown_in', 'planet_{}'.format(i)), self.zcrown_in/self.zplanet_in, p, self.axis, Axis(p,Z)),
			Gear(('crown_out', 'planet_{}'.format(i)), self.zcrown_out/self.zplanet_out, p, self.axis, Axis(p,Z)),
			])
	joints.append(Revolute(('sun', 'crown_in'), axis))
	return Kinematic(joints, ground='crown_in')
