import numpy as np
import math
<<<<<<< HEAD
#from scipy.integrate import solve_bvp as scipy_solve_bvp
import scikits.bvp_solver
=======
from scipy.integrate import solve_bvp as tpbvp
>>>>>>> b5585697bffa001b4f6911a40cdf0564b6cb3fda
import matplotlib.pyplot as plt
from utils import *

dt = 0.005

def ode_fun(tau, z):
    """
    This function computes the dz given tau and z. It is used in the bvp solver.
    Inputs:
        tau: (np.array shape [N,]) the independent variables. This must be the first argument.
        z: (np.array shape [n_z,N]) the state vectors. The first three states are z[:,i] = [x, y, th, ...]
    Output:
        dz: (np.array shape [n_z,N]) the state derivative vectors. Returns a numpy array.
    """
<<<<<<< HEAD
   ########## Code starts here ##########
    #Return array containing RHS of ODEs
    #z = [x1 x2 x_3 p1 p2 p3 r]
    u1 = (-0.5)*(z[3]*np.cos(z[2])+ z[4]*np.sin(z[2]))
    u2 = -(0.5)*z[5] 
    #r = z[6]  
    rdot = 0 #dummy state ode
    p1dot = 0
    p2dot = 0
    p3dot = u1*(z[3]*np.sin(z[2]) - z[4]*np.cos(z[2]))*z[6]
    #state odes
    xdot = u1*np.cos(z[2])* z[6]
    ydot = u1*np.sin(z[2]) *z[6]
    thdot =u2*z[6]
    
    #co-state odes
    dz  = np.array([xdot,ydot,thdot, p1dot, p2dot, p3dot,rdot])
=======
    (n_z, N) = np.shape(z)
    ########## Code starts here ##########

>>>>>>> b5585697bffa001b4f6911a40cdf0564b6cb3fda
    ########## Code ends here ##########
    return dz

def bc_fun(za, zb):
    """
    This function computes boundary conditions. It is used in the bvp solver.
    Inputs:
        za: the state vector at the initial time
        zb: the state vector at the final time
    Output:
        bc: tuple of boundary conditions
    """
    global lb  # declares lb = lambda as a global variable
    # final goal pose
    xf = 5
    yf = 5
    thf = -np.pi/2.0
    x_f = [xf, yf, thf]
    # initial pose
    x0 = [0, 0, -np.pi/2.0]

    ########## Code starts here ##########
    '''
    za = [x(0), y(0), th(0), p1(0), p2(0), p3(0), r(0)]
    zb = [x(tf), y(tf), th(0), p1(0), p2(0), p3(0), r(0)]
    Left boundary condition
    '''
    bca = np.array([za[0] - x0[0], za[1] - x0[1], za[2] - x0[2]])
    # Right boundary condition
    u1 = -0.5 * (zb[3] * np.cos(zb[2]) + zb[4] * np.sin(zb[2]))  # V
    u2 = -0.5 * zb[5]  # om
    h_b = lb + u1**2 + u2**2 + zb[3]*u1*np.cos(zb[2]) + zb[4]*u1*np.sin(zb[2]) + zb[5]*u2
    bcb = np.array([(zb[0] - x_f[0]), (zb[1] - x_f[1]), (zb[2] - x_f[2]), h_b])

    ########## Code ends here ##########
    return np.concatenate((bca, bcb))

def solve_bvp(problem_inputs, initial_guess):
    """
    This function solves the bvp_problem.
    Inputs:
        problem_inputs: a dictionary of the arguments needs to define the problem
                        fun, bc, x
        initial_guess: initial guess of the solution
    Output:
        z: a numpy array of the solution. It is of size [time, state_dim]

    Read this documentation -- https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_bvp.html
    """
    result = tpbvp(y=initial_guess, verbose=2, **problem_inputs)
    soln = result['sol']
    print(result['message'])

    # Test if time is reversed in bvp_solver solution
    flip, tf = check_flip(soln(0))
    t = np.arange(0,tf,dt)
    z = soln(t/tf)
    if flip:
        z[3:7,:] = -z[3:7,:]
    z = z.T # solution arranged so that it is [time, state_dim]
    return z

def compute_controls(z):
    """
    This function computes the controls V, om, given the state z. It is used in main().
    Input:
        z: z is the state vector for multiple time instances. It has size [time, state_dim]
    Outputs:
        V: velocity control input
        om: angular rate control input
    """
    ########## Code starts here ##########
    L = np.shape(z)[0]
    V = np.zeros(L)
    om = np.zeros(L)
    for i in range(L):
        V[i] = -0.5 * (z[i, 3]*np.cos(z[i, 2]) + z[i, 4]*np.sin(z[i, 2]))
        om[i] = -0.5 * z[i, 5]

    ########## Code ends here ##########

    return V, om

def main():
    """
    This function solves the specified bvp problem and returns the corresponding optimal contol sequence
    Outputs:
        V: optimal V control sequence 
        om: optimal om ccontrol sequence
    You are required to define the problem inputs, initial guess, and compute the controls

    Hint: The total time is between 15-25
    """
    ########## Code starts here ##########
    global lb  # declares lb = lambda as a global variable
    lb = .24
    num_ODE = 7
    num_parameters = 0
    num_left_boundary_conditions = 3
    boundary_points = (0,1)
    function = ode_fun
    boundary_conditions = bc_fun
    #initial_guess = np.array([2.5, 2.5, -np.pi/2.0, -2.0, -.10, 0.5, 20])
    initial_guess =  (2.5, 2.5, -np.pi/2.0, 0,  1,  -0.1, 15)
    ########## Code ends here ##########

    problem_inputs = {
                      'fun' : function,
                      'bc'  : boundary_conditions,
                      'x'   : tau,
                     }

    z = solve_bvp(problem_inputs, initial_guess)
    V, om = compute_controls(z)
    print("Vmax", max(abs(V)))
    print("om", max(abs(om)))
    return z, V, om

if __name__ == '__main__':
    z, V, om = main()
    tf = z[0,-1]
    t = np.arange(0,tf,dt)
    x = z[:,0]
    y = z[:,1]
    th = z[:,2]
    data = {'z': z, 'V': V, 'om': om}
    save_dict(data, 'data/optimal_control.pkl')
    maybe_makedirs('plots')

    # plotting
    # plt.rc('font', weight='bold', size=16)
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(x, y,'k-',linewidth=2)
    plt.quiver(x[1:-1:200], y[1:-1:200],np.cos(th[1:-1:200]),np.sin(th[1:-1:200]))
    plt.grid(True)
    plt.plot(0,0,'go',markerfacecolor='green',markersize=15)
    plt.plot(5,5,'ro',markerfacecolor='red', markersize=15)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis([-1, 6, -1, 6])
    plt.title('Optimal Control Trajectory')

    plt.subplot(1, 2, 2)
    plt.plot(t, V,linewidth=2)
    plt.plot(t, om,linewidth=2)
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.legend(['V [m/s]', '$\omega$ [rad/s]'], loc='best')
    plt.title('Optimal control sequence')
    plt.tight_layout()
    plt.savefig('plots/optimal_control.png')
    plt.show()
