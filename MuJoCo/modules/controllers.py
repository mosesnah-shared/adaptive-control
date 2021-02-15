# [Built-in modules]

# [3rd party modules]
import numpy as np
import time
import pickle

from modules.utils        import my_print
from MLmodules.RBFN       import RBFN


import matplotlib.pyplot as plt

try:
    import mujoco_py as mjPy

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

# Added
try:
    import sympy as sp
    from sympy.utilities.lambdify import lambdify, implemented_function

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install sympy, \
                                             Simply type pip3 install sympy. \
                                             Sympy is necessary for building ZFT Calculation)".format( e ) )

# [Local modules]


class Controller( ):
    """
        Description:
        -----------
            Parent class for the controllers
    """


    def __init__( self, mjModel, mjData ):
        """

        """
        self.mjModel        = mjModel
        self.mjData         = mjData
        self.ctrl_par_names = None


    def set_ctrl_par( self, **kwargs ):
        """
            Setting the control parameters

            Each controllers have their own controller parameters names (ctrl_par_names),

            This method function will become handy when we want to modify, or set the control parameters.

        """
        if kwargs is not None:
            for args in kwargs:
                if args in self.ctrl_par_names:
                    setattr( self, args, kwargs[ args ] )
                else:
                    pass

    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.


class AdaptiveController( Controller ):
    """
        Description:
        ----------
            Class for a Cartesian Impedance Controller

            [REF] Slotine, Jean-Jacques E., and Weiping Li. "On the adaptive control of robot manipulators." The international journal of robotics research 6.3 (1987): 49-59.APA

    """

    def __init__( self, mjModel, mjData, type = 1, is_robust = False ):

        super().__init__( mjModel, mjData )

        self.act_names      = mjModel.actuator_names
        self.n_act          = len( mjModel.actuator_names )
        self.idx_act        = np.arange( 0, self.n_act )
        self.n_limbs        = '-'.join( mjModel.body_names ).lower().count( 'arm' )
        self.g              = -mjModel.opt.gravity[ 2 ]

        self.type           = type                                              # If type 1, then Joint Space Planning
                                                                                # If type 2, then Cartesian Space Planning

        self.t_sym = sp.symbols( 't' )                                          # time symbol of the equation, useful when defining the trajectory


        self.NN = None                                                          # The neural network for compensation.

        # For the desired trajectory to be followed
        # Regardless of whether it is Cartesian Space/Joint Space tracking, we'll set the trajectory
        # self.traj_func_pos =
        # self.traj_func_vel =

        self.J_old, self.J_new = None, None
        self.dt     = self.mjModel.opt.timestep


        if   self.n_act == 2:
            # For 2DOF robot, size of a is 5, Y is 2-by-5
            self.a = np.zeros( 5 )
            self.Y = np.zeros( ( 2, 5 ) )

        elif self.n_act == 4:
            # For 4DOF robot, size of a is 11, Y is 4-by-11
            self.a = np.zeros( 11 )
            self.Y = np.zeros( ( 4, 11 ) )



    def set_trajectory( self, trajectory ):
        """
            Setting the trajectory of the simulation

            The name of the variable is traj_pos, traj_vel, traj_acc, to make it agnostic to Cartesian/Joint Space

            Prefix func is added to the variable to denote that the function is "lambda" function.
        """

        self.func_traj_pos = trajectory
        self.func_traj_vel = [ sp.diff( tmp, self.t_sym ) for tmp in self.func_traj_pos  ]
        self.func_traj_acc = [ sp.diff( tmp, self.t_sym ) for tmp in self.func_traj_vel  ]

        self.func_traj_pos = lambdify( self.t_sym, self.func_traj_pos )
        self.func_traj_vel = lambdify( self.t_sym, self.func_traj_vel )
        self.func_traj_acc = lambdify( self.t_sym, self.func_traj_acc )

        # Get the initial position of the q vector
        # self.traj_pos = self.func_traj_pos( 0 )
        # self.traj_vel = self.func_traj_vel( 0 )
        # self.traj_acc = self.func_traj_acc( 0 )

    def get_Y_and_s( self, q, dq, t ):

        self.traj_pos = self.func_traj_pos( t )                                 # First saving the trajectory position

        if  self.type == 1:                                                     # If joint space tracking task

            qd   = self.func_traj_pos( t )                                      # Just for simplification of the notation
            dqd  = self.func_traj_vel( t )                                      # Just for simplification of the notation
            ddqd = self.func_traj_acc( t )                                      # Just for simplification of the notation

            qtilde  = q -  qd
            dqtilde = dq - dqd

            self.Lgain = 20 * np.eye( self.n_act )
            self.dqr   = dqd  - self.Lgain.dot( qtilde  )
            self.ddqr  = ddqd - self.Lgain.dot( dqtilde )

            self.s = dq - self.dqr                                                   # The sliding variable.

        elif self.type == 2:
            # If Cartesian space tracking task

            xd   = self.func_traj_pos( t )                                      # Just for simplification of the notation
            dxd  = self.func_traj_vel( t )                                      # Just for simplification of the notation
            ddxd = self.func_traj_acc( t )                                      # Just for simplification of the notation

            # The name of EEGEOM is crucial for this task
            # Hence for the xml file, do not change the name EEGEOM for the end-effector
            xEE  = np.array(  self.mjData.get_geom_xpos( "EEGEOM" ) )           # Get the end-effector position
            dxEE = np.array( self.mjData.get_geom_xvelp( "EEGEOM" ) )           # Get the end-effector velocity
            JEE  = self.mjData.get_geom_jacp( "EEGEOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ]  # Get the end-effector Jacobian

            xtilde  = xEE  -  xd
            dxtilde = dxEE - dxd


            # [TODO] [Moses C. Nah] [2020.02.21]
            # For this controller, the Jacobian matrix is exactly known, but it actually doesn't make sense
            # since the adaptive controller doesn't know the geometrical details of the robot.
            # Hence, adding the Jacobian parameters for the adaptive controller too.

            if self.J_new is None:
                # If J_new is not initialized, then filling in with the jacobian
                self.J_new = JEE
                dJ         = np.zeros( ( 3, self.n_act ) )

            else:
                self.J_old = self.J_new
                self.J_new = JEE

                dJ = ( self.J_new - self.J_old ) / self.dt                      # Getting the time difference of Jacobian.


            if   self.n_act == 2:
                # If 2DOF robot model, then no movement in the y-direction since the movement is confined to the 2D sagittal plane
                xtilde  = np.delete( xtilde , 1, 0 )      # Erasing the y (2nd row) element to prevent singularity
                dxtilde = np.delete( dxtilde, 1, 0 )      # Erasing the y (2nd row) element to prevent singularity\
                dxd     = np.delete( dxd    , 1, 0 )      # Erasing the y (2nd row) element to prevent singularity
                ddxd    = np.delete( ddxd   , 1, 0 )      # Erasing the y (2nd row) element to prevent singularity

                JEE     = np.delete( JEE , 1, 0 )         # Erasing the element (2nd row) to prevent singularity
                dJ      = np.delete( dJ  , 1, 0 )         # Erasing the element (2nd row) to prevent singularity



            self.Lgain = 1.0 * np.eye( self.n_act )

            if   self.n_act == 2:
                self.Lgain = 1.0 * np.eye( self.n_act )
                Jinv        = np.linalg.inv( JEE )

            elif self.n_act == 4:
                self.Lgain  = 1.0 * np.eye( 3 )
                Jinv        = np.linalg.pinv( JEE )

            self.dqr  = Jinv.dot(  dxd -  self.Lgain.dot( xtilde   ) )
            self.ddqr = Jinv.dot( ddxd -  self.Lgain.dot( dxtilde  ) - dJ.dot( self.dqr ) )


            self.s    = dq - self.dqr                                                # Defining the sliding variable


        ddqr, dqr = self.ddqr, self.dqr                                         # Just for notation simplification

        if self.n_act == 2:     # For 2D Model Case

            self.Y[ 0, 0 ] = ddqr[ 0 ]
            self.Y[ 0, 1 ] = ddqr[ 0 ] + ddqr[ 1 ]
            self.Y[ 0, 2 ] =  2*ddqr[0]*np.cos(q[1]) + ddqr[1]*np.cos(q[1]) - dqr[0]*np.sin(q[1])*dq[1] - dqr[1]*np.sin(q[1])*dq[0] - dqr[1]*np.sin(q[1])*dq[1]
            self.Y[ 0, 3 ] = np.sin(q[0])
            self.Y[ 0, 4 ] = np.sin( q[0] + q[1] )
            #
            self.Y[ 1, 0 ] = 0
            self.Y[ 1, 1 ] = ddqr[ 0 ] + ddqr[ 1]
            self.Y[ 1, 2 ] = ddqr[0]*np.cos(q[1]) + dqr[0]*np.sin(q[1])*dq[0]
            self.Y[ 1, 3 ] = 0
            self.Y[ 1, 4 ] = np.sin( q[0] + q[1] )


        elif self.n_act == 4:   # For 3D Model Case

            self.Y[0, 0] = ddqr[0]*np.cos(q[1])**2*np.sin(q[2])**2 - (dqr[0]*np.sin(2*q[1])*dq[1])/2 - (dqr[1]*np.sin(2*q[1])*dq[0])/2 - (dqr[1]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[1])/2 + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[1] + ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) - dqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] +                    dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[0, 1] = ddqr[0]*np.sin(q[1])**2*np.sin(q[3])**2 + (dqr[0]*np.sin(2*q[1])*dq[1])/2 + (dqr[1]*np.sin(2*q[1])*dq[0])/2 + (dqr[0]*np.sin(2*q[3])*dq[3])/2 + (dqr[3]*np.sin(2*q[3])*dq[0])/2 - ddqr[2]*np.sin(q[1])*np.sin(q[3])**2 - (dqr[1]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 - (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 + (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 + (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 + ddqr[0]*np.cos(q[1])**2*np.cos(q[3])**2*np.sin(q[2])**2 + ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - ddqr[2]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2]) - dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] - dqr[3]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*dqr[1]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*dqr[3]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] + 2*dqr[1]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + 2*dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + 2*ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] - dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1]

            self.Y[0, 2] = ddqr[0]*np.cos(q[1])**2*np.cos(q[2])**2 + (dqr[1]*np.cos(q[1])*dq[2])/2 + (dqr[2]*np.cos(q[1])*dq[1])/2 - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) + dqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[0, 3] = ddqr[0]*np.cos(q[1])**2*np.cos(q[2])**2 + ddqr[3]*np.cos(q[1])*np.cos(q[2]) + (dqr[1]*np.cos(q[1])*dq[2])/2 + (dqr[2]*np.cos(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 - (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) + dqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[0, 4] = ddqr[0]*np.sin(q[1])**2 - ddqr[2]*np.sin(q[1]) + (dqr[0]*np.sin(2*q[1])*dq[1])/2 + (dqr[1]*np.sin(2*q[1])*dq[0])/2 - (dqr[1]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[1])/2

            self.Y[0, 5] = ddqr[0]*np.cos(q[3])**2*np.sin(q[1])**2 - (dqr[0]*np.sin(2*q[1])*dq[1])/2 - (dqr[1]*np.sin(2*q[1])*dq[0])/2 - (dqr[0]*np.sin(2*q[3])*dq[3])/2 - (dqr[3]*np.sin(2*q[3])*dq[0])/2 - ddqr[2]*np.cos(q[3])**2*np.sin(q[1]) - (dqr[1]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[1])/2 + (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[1] + ddqr[0]*np.cos(q[1])**2*np.sin(q[2])**2*np.sin(q[3])**2 - ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + ddqr[2]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*np.sin(q[3])**2 + dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] - dqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] + dqr[3]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*dqr[1]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*dqr[3]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] + dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] - 2*dqr[1]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - 2*dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - 2*ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3]) - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] + dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1]

            self.Y[0, 6] = ddqr[0]/2 + (ddqr[0]*np.cos(2*q[1]))/2 - (dqr[0]*np.sin(2*q[1])*dq[1])/2 - (dqr[1]*np.sin(2*q[1])*dq[0])/2

            self.Y[0, 7] = 2*ddqr[0]*np.cos(q[1])**2*np.cos(q[3]) - dqr[0]*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])**2*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])**2*np.sin(q[3])*dq[0] + ddqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3]) + ddqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[3]) - ddqr[2]*np.cos(q[1])*np.sin(q[2])*np.sin(q[3]) + 2*ddqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3]) - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[1] - 2*dqr[1]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[0] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[1] - dqr[2]*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[2] - dqr[3]*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[3] + 2*dqr[0]*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[1] + 2*dqr[1]*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[3] + dqr[3]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[0]

            self.Y[0, 8] = ddqr[0]*np.cos(q[1])**2*np.cos(q[2])**2 + ddqr[0]*np.sin(q[1])**2*np.sin(q[3])**2 + (dqr[0]*np.sin(2*q[1])*dq[1])/2 + (dqr[1]*np.sin(2*q[1])*dq[0])/2 + (dqr[0]*np.sin(2*q[3])*dq[3])/2 + (dqr[3]*np.sin(2*q[3])*dq[0])/2 + ddqr[3]*np.cos(q[1])*np.cos(q[2]) - ddqr[2]*np.sin(q[1])*np.sin(q[3])**2 - dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[3] - dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) + ddqr[0]*np.cos(q[1])**2*np.cos(q[3])**2*np.sin(q[2])**2 + ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - ddqr[2]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + ddqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2]) - dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] + dqr[1]*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - dqr[3]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*dqr[1]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*dqr[3]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] + 2*dqr[1]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + 2*dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + 2*ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] - dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1]

            self.Y[0, 9] = np.cos(q[1])*np.sin(q[0])

            self.Y[0, 10] = np.cos(q[1])*np.cos(q[3])*np.sin(q[0]) + np.cos(q[0])*np.cos(q[2])*np.sin(q[3]) + np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])

            self.Y[1, 0] = ddqr[1]*np.cos(q[2])**2 + (dqr[0]*np.sin(2*q[1])*dq[0])/2 - (dqr[1]*np.sin(2*q[2])*dq[2])/2 - (dqr[2]*np.sin(2*q[2])*dq[1])/2 - (dqr[0]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[0])/2 + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[0] + ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0]

            self.Y[1, 1] = ddqr[1]*np.cos(q[2])**2*np.cos(q[3])**2 - (dqr[0]*np.sin(2*q[1])*dq[0])/2 + (dqr[0]*np.cos(q[1])*dq[2])/2 + (dqr[2]*np.cos(q[1])*dq[0])/2 + (dqr[2]*np.cos(q[2])*dq[3])/2 + (dqr[3]*np.cos(q[2])*dq[2])/2 - (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 - (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[3])**2*dq[0] - dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[2] - ddqr[2]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) + ddqr[0]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2]) + dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[2]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] - dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[1, 2] = ddqr[1]*np.sin(q[2])**2 + (dqr[1]*np.sin(2*q[2])*dq[2])/2 + (dqr[2]*np.sin(2*q[2])*dq[1])/2 + (dqr[0]*np.cos(q[1])*dq[2])/2 + (dqr[2]*np.cos(q[1])*dq[0])/2 - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[0] - ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0]

            self.Y[1, 3] = ddqr[1] - ddqr[3]*np.sin(q[2]) - ddqr[1]*np.cos(q[2])**2 + (dqr[1]*np.sin(2*q[2])*dq[2])/2 + (dqr[2]*np.sin(2*q[2])*dq[1])/2 + (dqr[0]*np.cos(q[1])*dq[2])/2 + (dqr[2]*np.cos(q[1])*dq[0])/2 - (dqr[2]*np.cos(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[2])*dq[2])/2 + (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[0] - ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0]

            self.Y[1, 4] = (dqr[0]*np.cos(q[1])*dq[2])/2 - (dqr[0]*np.sin(2*q[1])*dq[0])/2 + (dqr[2]*np.cos(q[1])*dq[0])/2

            self.Y[1, 5] = ddqr[1]*np.cos(q[2])**2 - ddqr[1]*np.cos(q[2])**2*np.cos(q[3])**2 + (dqr[0]*np.sin(2*q[1])*dq[0])/2 - (dqr[1]*np.sin(2*q[2])*dq[2])/2 - (dqr[2]*np.sin(2*q[2])*dq[1])/2 - (dqr[0]*np.cos(q[1])*dq[2])/2 - (dqr[2]*np.cos(q[1])*dq[0])/2 - (dqr[2]*np.cos(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[2])*dq[2])/2 + (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (dqr[3]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[3])**2*dq[0] + dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[2] + ddqr[2]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) - ddqr[0]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*np.sin(q[3])**2 - dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[2]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] + dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[1, 6] = ddqr[1] + (dqr[0]*np.sin(2*q[1])*dq[0])/2

            self.Y[1, 7] = 2*ddqr[1]*np.cos(q[3]) - ddqr[2]*np.cos(q[2])*np.sin(q[3]) - ddqr[3]*np.cos(q[3])*np.sin(q[2]) - dqr[1]*np.sin(q[3])*dq[3] - dqr[3]*np.sin(q[3])*dq[1] - dqr[2]*np.cos(q[2])*np.cos(q[3])*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])*dq[2] + dqr[0]*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[2]*np.sin(q[2])*np.sin(q[3])*dq[2] + dqr[3]*np.sin(q[2])*np.sin(q[3])*dq[3] + ddqr[0]*np.cos(q[2])*np.sin(q[1])*np.sin(q[3]) + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[0] - dqr[0]*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] - dqr[2]*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - 2*dqr[0]*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[1, 8] = ddqr[1] - ddqr[3]*np.sin(q[2]) - ddqr[1]*np.cos(q[2])**2 + ddqr[1]*np.cos(q[2])**2*np.cos(q[3])**2 - (dqr[0]*np.sin(2*q[1])*dq[0])/2 + (dqr[1]*np.sin(2*q[2])*dq[2])/2 + (dqr[2]*np.sin(2*q[2])*dq[1])/2 + dqr[0]*np.cos(q[1])*dq[2] + dqr[2]*np.cos(q[1])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[2])**2*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[3])**2*dq[0] - dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[2] - ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[2]) - ddqr[2]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) + ddqr[0]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2]) + dqr[0]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[2]*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] - dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[1, 9] = np.cos(q[0])*np.sin(q[1])

            self.Y[1, 10] = np.cos(q[0])*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3]))

            self.Y[2, 0] = (dqr[0]*np.cos(q[1])*dq[1])/2 + (dqr[1]*np.cos(q[1])*dq[0])/2 + dqr[1]*np.cos(q[2])*np.sin(q[2])*dq[1] - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[2, 1] = ddqr[2]*np.sin(q[3])**2 + (dqr[2]*np.sin(2*q[3])*dq[3])/2 + (dqr[3]*np.sin(2*q[3])*dq[2])/2 - ddqr[0]*np.sin(q[1])*np.sin(q[3])**2 - (dqr[0]*np.cos(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[1])*dq[0])/2 + (dqr[1]*np.cos(q[2])*dq[3])/2 + (dqr[3]*np.cos(q[2])*dq[1])/2 + (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 + (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[3])**2*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[1] - ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) - ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0]

            self.Y[2, 2] = dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - (dqr[1]*np.cos(q[1])*dq[0])/2 - dqr[1]*np.cos(q[2])*np.sin(q[2])*dq[1] - (dqr[0]*np.cos(q[1])*dq[1])/2 + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[2, 3] = (dqr[1]*np.cos(q[2])*dq[3])/2 - (dqr[0]*np.cos(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[1])*dq[0])/2 - (dqr[1]*np.sin(2*q[2])*dq[1])/2 + (dqr[3]*np.cos(q[2])*dq[1])/2 + (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 + (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0]

            self.Y[2, 4] = ddqr[2] - ddqr[0]*np.sin(q[1]) - (dqr[0]*np.cos(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[1])*dq[0])/2

            self.Y[2, 5] = ddqr[2]*np.cos(q[3])**2 + (dqr[1]*np.sin(2*q[2])*dq[1])/2 - (dqr[2]*np.sin(2*q[3])*dq[3])/2 - (dqr[3]*np.sin(2*q[3])*dq[2])/2 - ddqr[0]*np.cos(q[3])**2*np.sin(q[1]) + (dqr[0]*np.cos(q[1])*dq[1])/2 + (dqr[1]*np.cos(q[1])*dq[0])/2 - (dqr[1]*np.cos(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[2])*dq[1])/2 - (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[0])/2 - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[3])**2*dq[0] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[3] + dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[1] + ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) + ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] + dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0]

            self.Y[2, 6] = 0

            self.Y[2, 7] = -np.sin(q[3])*(ddqr[1]*np.cos(q[2]) + ddqr[0]*np.cos(q[1])*np.sin(q[2]) - dqr[0]*np.sin(q[1])*np.sin(q[2])*dq[1] - dqr[1]*np.sin(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*dq[0])

            self.Y[2, 8] = ddqr[2]*np.sin(q[3])**2 - (dqr[1]*np.sin(2*q[2])*dq[1])/2 + (dqr[2]*np.sin(2*q[3])*dq[3])/2 + (dqr[3]*np.sin(2*q[3])*dq[2])/2 - ddqr[0]*np.sin(q[1])*np.sin(q[3])**2 - dqr[0]*np.cos(q[1])*dq[1] - dqr[1]*np.cos(q[1])*dq[0] + dqr[1]*np.cos(q[2])*dq[3] + dqr[3]*np.cos(q[2])*dq[1] + dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[3] + dqr[3]*np.cos(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])**2*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])**2*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[3])**2*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[3] - dqr[3]*np.cos(q[2])*np.cos(q[3])**2*dq[1] - ddqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[3]) - ddqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3]) - dqr[0]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - dqr[3]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - dqr[3]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0]

            self.Y[2, 9] = 0

            self.Y[2, 10] = -np.sin(q[3])*(np.sin(q[0])*np.sin(q[2]) + np.cos(q[0])*np.cos(q[2])*np.sin(q[1]))

            self.Y[3, 0] = 0

            self.Y[3, 1] = (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (dqr[2]*np.sin(2*q[3])*dq[2])/2 - (dqr[1]*np.cos(q[2])*dq[2])/2 - (dqr[2]*np.cos(q[2])*dq[1])/2 - (dqr[0]*np.sin(2*q[3])*dq[0])/2 + (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[1] + dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[3, 2] = 0

            self.Y[3, 3] = ddqr[3] - ddqr[1]*np.sin(q[2]) + ddqr[0]*np.cos(q[1])*np.cos(q[2]) - (dqr[1]*np.cos(q[2])*dq[2])/2 - (dqr[2]*np.cos(q[2])*dq[1])/2 - (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[0])/2

            self.Y[3, 4] = 0

            self.Y[3, 5] = (dqr[0]*np.sin(2*q[3])*dq[0])/2 + (dqr[2]*np.sin(2*q[3])*dq[2])/2 + (dqr[1]*np.cos(q[2])*dq[2])/2 + (dqr[2]*np.cos(q[2])*dq[1])/2 - (dqr[0]*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (dqr[1]*np.cos(q[2])*np.sin(q[1])*dq[0])/2 + (dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[2])/2 + (dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[0])/2 - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[2] - dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[1] - dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] + 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[3, 6] = 0

            self.Y[3, 7] = dqr[1]*np.sin(q[3])*dq[1] - ddqr[1]*np.cos(q[3])*np.sin(q[2]) + dqr[0]*np.cos(q[1])**2*np.sin(q[3])*dq[0] + ddqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3]) - dqr[0]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[1] - dqr[1]*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[0] - dqr[0]*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[0]

            self.Y[3, 8] = ddqr[3] - ddqr[1]*np.sin(q[2]) - (dqr[0]*np.sin(2*q[3])*dq[0])/2 - (dqr[2]*np.sin(2*q[3])*dq[2])/2 + ddqr[0]*np.cos(q[1])*np.cos(q[2]) - dqr[1]*np.cos(q[2])*dq[2] - dqr[2]*np.cos(q[2])*dq[1] - dqr[0]*np.cos(q[1])*np.sin(q[2])*dq[2] - dqr[2]*np.cos(q[1])*np.sin(q[2])*dq[0] + dqr[1]*np.cos(q[2])*np.cos(q[3])**2*dq[2] + dqr[2]*np.cos(q[2])*np.cos(q[3])**2*dq[1] + dqr[0]*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + dqr[0]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + dqr[2]*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - dqr[0]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*dqr[0]*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - dqr[1]*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + dqr[2]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + dqr[1]*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - 2*dqr[0]*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - dqr[0]*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + dqr[0]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + dqr[1]*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]

            self.Y[3, 9] = 0

            self.Y[3, 10] = np.cos(q[0])*np.cos(q[1])*np.sin(q[3]) + np.cos(q[2])*np.cos(q[3])*np.sin(q[0]) - np.cos(q[0])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])

        tmpY = self.Y.T
        tmps = self.s.T

        if   self.n_act == 2:
            self.gamma = np.diag( [ 1,1,1, 2000, 2000 ] )

        elif self.n_act == 4:
            self.gamma = np.diag( [ 2.5,2.5, 2.5,2.5,2.5,2.5, 2.5, 2.5,2.5, 100, 100 ] )

        self.a += -self.dt * self.gamma.dot( tmpY.dot( tmps ).T  )
        return self.Y, self.s

    def input_calc( self, start_time, current_time ):
        """

        """

        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]

        t0  = current_time - start_time

        if   self.n_act == 2:
            self.kd = 200 * np.eye( self.n_act )

        elif self.n_act == 4:
            self.kd = 10  * np.eye( self.n_act )

        Y, s =  self.get_Y_and_s( q, dq, t0 )

        if self.NN:
            self.tau_d = self.NN.calc_activation( s )
            self.NN.update_weight( s, self.dt )

        else:
            self.tau_d = np.zeros( self.n_act )

        self.tau =  Y.dot( self.a.T ) - self.kd.dot( s.T )

        return self.mjData.ctrl, self.idx_act, self.tau

    def add_NN( self, type = "RBFNN" ):
        """
            Add a neural network to this controller
        """

        #  Number of hidden layers, number of actuators.
        self.NN = RBFN(  self.n_act, 40,   self.n_act )

if __name__ == "__main__":
    pass
