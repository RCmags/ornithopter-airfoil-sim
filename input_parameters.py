# aerodynamics
COEF_AERO    = 1.5           # aerodynamic force scale
VEL_INF      = 1             # velocity of incoming flow
CHORD        = 1             # airfoil chord
COEF_AERO   *= VEL_INF**2

# mass parameters
MASS_POS     = -0.5          # position of center of mass
MASS         = 0.3           # mass of airfoil
INERTIA      = MASS*( (CHORD**2)/12 + MASS_POS**2 )   # assume airfoil is thin rod

# oscillation
AMPLITUDE    = 1             # amplitude of oscillation
ANG_FREQ     = 0.8           # angular frequency of oscillation

# angular spring
COEF_SPRING  = 0.2           # angular spring/elastic constant
COEF_DAMPING = 0.1           # angular damping constant
ANGLE_BIAS   = 0.4           # angle offset in rest point of angular spring

# animation
TIME_MAX     = 40            # maximum time value for simulation
FRAME_TIME   = 50            # frame delay in milliseconds
SAVE_FIGURES = False         # save animation as .gif and plot as .jpg

# axis bounds
XLIM         = 1.5
YLIM         = 2
