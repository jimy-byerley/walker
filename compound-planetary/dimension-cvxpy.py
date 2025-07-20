import cvxpy as cp
from math import *

def ratio(zs, ze2, zp2, zp1):
	return zs / (2*ze2) * ((ze2 - zp2) / (zs + zp1) - zp2/zp1)

target = -60
rext = 50
min_step = 1

zs = cp.Variable(integer=True)
ze2 = cp.Variable(integer=True)
zp2 = cp.Variable(integer=True)
zp1 = cp.Variable(integer=True)

prob = cp.Problem(
	objective = cp.Minimize(cp.sum_squares(ratio(zs, ze2, zp2, zp1) - target)),
	constraints = [
		7 <= zs, zs <= floor(2*pi*rext/min_step),
		21 <= ze2, ze2 <= floor(2*pi*rext/min_step),
		7 <= zp2, zp2 <= 11,
		7 <= zp1, zp1 <= 11,
		],
	)

prob.solve()
print(zs.value, ze2.value, zp2.value, zp1.value)
