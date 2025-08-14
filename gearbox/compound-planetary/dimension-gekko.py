from gekko import GEKKO
from math import *

def ratio(zs, ze2, zp2, zp1):
	return zs / (2*ze2) * ((ze2 - zp2) / (zs + zp1) - zp2/zp1)

target = 1/8
rext = 50
min_step = 4
zmin = 7

m = GEKKO()
m.options.SOLVER=1  # APOPT is an MINLP solver
# optional solver settings with APOPT
m.solver_options = ['minlp_maximum_iterations 500',
                    # minlp iterations with integer solution
                    'minlp_max_iter_with_int_sol 10',
                    # treat minlp as nlp
                    'minlp_as_nlp 0',
                    # nlp sub-problem max iterations
                    'nlp_maximum_iterations 50',
                    # 1 = depth first, 2 = breadth first
                    'minlp_branch_method 1',
                    # maximum deviation from whole number
                    'minlp_integer_tol 0.05',
                    # covergence tolerance
                    'minlp_gap_tol 0.01']

print('zmin', zmin)
print('zmax', floor(2*pi*rext/min_step))
zs = m.Var(15, lb=zmin, ub=floor(2*pi*0.8*rext/min_step), integer=True)
ze2 = m.Var(40, lb=3*zmin, ub=floor(2*pi*0.7*rext/min_step), integer=True)
zp2 = m.Var(zmin, lb=zmin, ub=11, integer=True)
zp1 = m.Var(zmin, lb=zmin, ub=11, integer=True)

# desired reduction ratio
m.Equation(ratio(zs, ze2, zp2, zp1) == target)
# ensure the out planets do not cross
m.Equation(ze2 > 2*zp2)
# prefer small planets
m.Obj((zp1/(zs+2*zp1))**2)
m.Obj((zp2/ze2)**2)

m.solve(disp=False)
print(zs.value, ze2.value, zp2.value, zp1.value)
print(1/ratio(zs.value[0], ze2.value[0], zp2.value[0], zp1.value[0]))
