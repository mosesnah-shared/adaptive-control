

import numpy as np


class RBFN( object ):
    """
        Description:
        ----------
            Radial Basis Function, Neural Network, which will be added to our adaptive controller.
            For function fitting and etc.


    """

    def __init__( self, n_input, n_hidden, n_output ):
        """
        Arguments
        ----------
            [1] n_input:  Number of input layer neurons
            [2] n_hidden: Number of middle layer neurons
            [3] n_output: Number of output of the neurons

        Descriptions
        ----------
            The input vector goes into the n_input array. This input vector activates each n_hidden neurons.
            The activation is proportional to exp( -( input - center )^2 ), which makes it a RBFNN.
            This activation value is weighted with the weighting matrix, hence outputs the value.


        """

        # We don't need to specify the n_input, since the input is just a single vector which can be calculated directly.
        self.n_input  = n_input
        self.n_hidden = n_hidden
        self.n_output = n_output

        # The number of centers = # of inputs vs. # of hidden layers
        # [REF] Liu, Qiong, et al. "Enhanced PID: Adaptive Feedforward RBF Neural Network Control of Robot manipulators with an Optimal Distribution of Hidden Nodes." arXiv preprint arXiv:2005.11501 (2020).
        # The center matrix is n_input x n_hidden
        np.random.seed( 0 )

        # Distributing the radial basis functions around the square-space
        L = 1   # Upper/Lower limit of the range.
        self.centers  = np.random.uniform( low =    -L, high =    L, size = (  self.n_input, self.n_hidden  ) )
        self.vars     = np.random.uniform( low =   L/2, high =  L/2, size = ( self.n_hidden                 ) )
        self.weights  = np.random.uniform( low = -0.01, high = 0.01, size = ( self.n_hidden, self.n_output  ) )     # Weights are added between hidden + output layer.

        self.activation = np.zeros( n_hidden )
        self.gain       = 5*1e-2 * np.eye( self.n_hidden )

        # number of weights correspond to the number of basis functions

    def calc_activation( self, input ):

        # if self.n_input != len( input ):
        #     raise Exception( "Wrong input size, input size must be %d but %d is given", self.n_input, len( input ) )
        tmp = np.transpose( self.centers ) - input

        self.activation = - np.sum( np.abs( tmp )**2, axis = -1 ) / self.vars
        self.activation = np.exp( self.activation )

        return np.transpose( self.weights ).dot( self.activation )


    def update_weight( self, s, dt ):
        # [REF] Van Cuong, Pham, and Wang Yao Nan. "Adaptive trajectory tracking neural network control with robust compensator for robot manipulators." Neural Computing and Applications 27.2 (2016): 525-536.
        # The update rule is as follows:
        # W^{hat} = activation * s^{T}
        # [TIP] if you use simply dot, or product, then two vectors' inner product will come out, need to use outer
        self.weights += self.gain.dot( np.outer( self.activation,  s )  )
