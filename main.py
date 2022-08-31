#---------- Libraries: 
# math
import numpy as np
from math import pi as PI
from scipy.integrate import solve_ivp
# animation
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch

# include files
from input_parameters import *

#========= Functions ==========

#--------- airfoil shape

def rotateVector(x, y, angle):
	rot = np.array([ [np.cos(angle), -np.sin(angle)],
	                 [np.sin(angle),  np.cos(angle)] ])
	return np.dot(rot, [x, y])

def generateAirfoil():
	x = np.linspace(0, PI*0.5, num=50)
	x = np.sin(x)
	y = (x - 0.6*(x**3) - 0.3*(x**20) - 0.1*(x**200)) * 0.1 # multiple is thickness
	x = x - 1
	return x*CHORD, y*CHORD

	# airfoil coordinates
XFOIL, YFOIL = generateAirfoil()

def airfoilPoints(angle):
	xt, yt = rotateVector(XFOIL, YFOIL, angle)
	xb, yb = rotateVector(XFOIL,-YFOIL, angle)
	return xt, yt, xb, yb 
	
#---------- figure rotation

def rotateAirfoil(angle, y):
	xt, yt = rotateVector(XFOIL, YFOIL, angle)
	xb, yb = rotateVector(XFOIL,-YFOIL, angle)
	airfoil_t.set_data(xt, yt+y)
	airfoil_b.set_data(xb, yb+y)

def setVelocity(y, ydot):
	p1 = (0, y)
	p2 = (VEL_INF, ydot+y)
	arrow_v.set_positions(p1, p2)
	label_v.set_position(p2)
	center_v.center = p1

def setLift(fx, fy, angle, y):
	cx, cy = rotateVector( [-0.25*CHORD], [0], angle)
	p1 = np.array( [cx[0], cy[0]+y] )
	p2 = p1 + np.array( [fx, fy] )
	arrow_l.set_positions( p1, p2 ) 
	label_l.set_position(p2)
	center_l.center = p1

def setAccel(fy, angle, y):
	cx, cy = rotateVector( [MASS_POS], [0], angle)
	p1 = np.array( [cx[0], cy[0]+y] )
	p2 = p1 + np.array( [0, fy] ) 	
	arrow_a.set_positions( p1, p2 )
	label_a.set_position(p2)
	center_a.center = p1

#---------- animation functions

def anim_init():
	airfoil_t.set_data(XFOIL, YFOIL)
	airfoil_b.set_data(XFOIL,-YFOIL)
	axis.add_patch(arrow_v)
	axis.add_patch(arrow_l)
	axis.add_patch(arrow_a)
	axis.add_patch(center_v)
	axis.add_patch(center_l)
	axis.add_patch(center_a)
	
	return airfoil_t, airfoil_b,        \
	       arrow_v, arrow_l, arrow_a,   \
	       center_v, center_a, center_l,\
	       label_v, label_a, label_l 

def anim_func(i):
	rotateAirfoil( angle_sol[i], y0[i] )
	
	setVelocity( y0[i], y1[i] )
	
	setLift( faero_x[i], faero_y[i], angle_sol[i], y0[i] )
	
	setAccel( facc_y[i], angle_sol[i], y0[i] )
	
	return airfoil_t, airfoil_b,        \
	       arrow_v, arrow_l, arrow_a,   \
	       center_v, center_a, center_l,\
	       label_v, label_a, label_l 

#---------- system solution

def inputDerivs(t):
	x = t * ANG_FREQ
	y0 = np.sin(x)*AMPLITUDE
	y1 = np.cos(x)*AMPLITUDE * ANG_FREQ
	y2 =-np.sin(x)*AMPLITUDE * ANG_FREQ**2
	return y0, y1, y2

def sumTorques(ang, angvel, t):
	# input values
	y0, y1, y2 = inputDerivs(t)
	cos_ang = np.cos(ang)
	sin_ang = np.sin(ang)
	
	# lift and drag
	vind = -cos_ang * (CHORD/4)*angvel         # rotation velocity
	ang_v = np.arctan2(y1 + vind, VEL_INF)    # effective velocity angle
	cos_angv = np.cos(ang_v)
	sin_angv = np.sin(ang_v)
		# angle of attack
	alpha = ang - ang_v
	cos_alpha = np.cos(alpha)
	sin_alpha = np.sin(alpha)
		# magnitudes
	lift = COEF_AERO * sin_alpha * cos_alpha
	drag = COEF_AERO * sin_alpha**2 
		# force components
	lx = -sin_angv * lift
	ly = cos_angv * lift
	dx = -cos_angv * drag
	dy = -sin_angv * drag
	faero_x = lx + dx
	faero_y = ly + dy
		# torque
	torque_aero = CHORD*( -cos_ang*(ly*0.25 + dy*0.5) + sin_ang*(lx*0.25 + dx*0.5) )

	# acceleration
	facc_y = -MASS*y2
	torque_acc = MASS_POS * cos_ang * facc_y

	# angular spring
	torque_spring = -COEF_SPRING*(ang-ANGLE_BIAS) - COEF_DAMPING*angvel
	
	# net torque
	torque_net = torque_aero + torque_acc + torque_spring

	return torque_net, faero_x, faero_y, facc_y, y0, y1, y2

def stateDeriv(t, x):
	dx1 = x[1]
	torque = sumTorques(x[0], x[1], t)[0]
	dx2 = torque / INERTIA
	return [dx1, dx2]

def solveSystem():
	state = solve_ivp(stateDeriv, (0, TIME_MAX), [0,0], method='Radau')
	time = state.t
	angle = state.y[0]
	angvel = state.y[1]
	return angle, angvel, time

#========= State variables ==========

#--------- animation figure

# - figure
figure, axis = plt.subplots(nrows=1, ncols=1)

	# format
axis.set_xlim(-XLIM, XLIM)
axis.set_ylim(-YLIM, YLIM)
axis.set_aspect('equal')

axis.set_xlabel("x-axis")
axis.set_ylabel("y-axis")
axis.spines['top'].set_visible(False)
axis.spines['right'].set_visible(False)

# - objects
airfoil_t, = axis.plot( [], [], color='black' )
airfoil_b, = axis.plot( [], [], color='black' )

arrow_v = FancyArrowPatch( (0,0), (0,0), color='magenta', mutation_scale=10, alpha=0.4 )
arrow_l = FancyArrowPatch( (0,0), (0,0), color='blue', mutation_scale=10, alpha=0.4 )
arrow_a = FancyArrowPatch( (0,0), (0,0), color='red', mutation_scale=10, alpha=0.4 )

center_v = plt.Circle( (0,0), 0.02, color ='green', alpha=0.5 )
center_l = plt.Circle( (0,0), 0.02, color ='blue', alpha=0.5 )
center_a = plt.Circle( (0,0), 0.02, color ='red', alpha=0.5 )

label_v = plt.text( VEL_INF   , 0, "v", color='magenta')
label_l = plt.text( CHORD*0.25, 0, "f", color='blue')
label_a = plt.text( MASS_POS  , 0, "-ma", color='red', fontsize='small')
	
#--------- compute motion

# ODE solution	
angle_sol, angvel_sol, time = solveSystem()

# forces 
torque_net, faero_x, faero_y, facc_y, y0, y1, y2 = sumTorques(angle_sol, angvel_sol, time)

fmean_x = np.average(faero_x)
fmean_y = np.average(faero_y)

#--------- display figures 

# 1. animation
anim = animation.FuncAnimation( figure, anim_func, init_func = anim_init, 
                                interval=FRAME_TIME, frames=len(time), blit = True )                       

# 2. aerodynamic force
figure1, (axis1, axis2) = plt.subplots(nrows=2, ncols=1, sharex=True)

	# x-component
axis1.plot(time, faero_x, color='red')
axis1.hlines(fmean_x, 0, TIME_MAX, color='black', linestyle='dashed')
	# y-component
axis2.plot(time, faero_y, color='blue')
axis2.hlines(fmean_y, 0, TIME_MAX, color='black', linestyle='dashed', label='mean')
axis2.legend()

	# format
axis1.set_title("Aerodynamic force")
axis1.set_ylabel("x-component")
axis2.set_xlabel("time")
axis2.set_ylabel("y-component")

# 3. display
if SAVE_FIGURES:
	anim.save("figures/airfoil_motion.gif", writer=animation.PillowWriter(fps=10), dpi=100 ) 
	figure1.savefig('figures/mean_force.jpg')

plt.tight_layout()
plt.show()
