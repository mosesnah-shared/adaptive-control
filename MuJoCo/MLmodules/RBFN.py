"""
File: RBFN.py
Author: Octavio Arriaga
Email: arriaga.camargo@email.com
Github: https://github.com/oarriaga
Description: Minimal implementation of a radial basis function network
"""

import numpy as np


class RBFN( object ):

    def __init__( self, n_input, n_hidden, n_output ):
        """ radial basis function network
        # Arguments
            input_shape: dimension of the input data
            e.g. scalar functions have should have input_dimension = 1
            hidden_shape: the number of hidden radial basis functions,
            also, number of centers.
        """
        self.n_input  = n_input
        self.n_hidden = n_hidden
        self.n_output = n_output

        # The number of centers = # of inputs vs. # of hidden layers
        # [REF] Liu, Qiong, et al. "Enhanced PID: Adaptive Feedforward RBF Neural Network Control of Robot manipulators with an Optimal Distribution of Hidden Nodes." arXiv preprint arXiv:2005.11501 (2020).
        #
        # The center matrix is n_input x n_hidden

        np.random.seed( 0 )
        L = 1
        self.centers  = np.random.uniform( low = -L, high = L, size = ( self.n_hidden, self.n_input  ) )
        self.vars     = np.random.uniform( low =  L/2,
                                          high =  L/2,
                                          size = ( self.n_hidden )   )                           # Number corresponds to the number of neurons
        self.weights  = np.random.uniform( low = -0.01, high = 0.01, size = ( self.n_hidden, self.n_output  ) )

        self.activation = np.zeros( n_hidden )
        self.gain       = 5*1e-2 * np.eye( self.n_hidden )

        # number of weights correspond to the number of basis functions
    def calc_activation( self, input ):

        # if self.n_input != len( input ):
        #     raise Exception( "Wrong input size, input size must be %d but %d is given", self.n_input, len( input ) )

        self.activation = - np.sum( np.abs( self.centers - input )**2, axis = -1 ) / self.vars
        self.activation = np.exp( self.activation )

        return np.transpose( self.weights ).dot( self.activation )


    def update_weight( self, s, dt ):
        # [REF] Van Cuong, Pham, and Wang Yao Nan. "Adaptive trajectory tracking neural network control with robust compensator for robot manipulators." Neural Computing and Applications 27.2 (2016): 525-536.
        # The update rule is as follows:
        # W^{hat} = activation * s^{T}
        # [TIP] if you use simply dot, or product, then two vectors' inner product will come out, need to use outer
        self.weights += self.gain.dot( np.outer( self.activation,  s )  )
