classdef my2DOFRobotAndRod < handle
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an 4-DOF planar robot, superclass my2DOFRobot
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     21-October-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
    end
        
    properties ( SetAccess = public )
        
        
                        % [Symbolic values]
                        % [DESCRIPTIONS]                         [SIZE]
        t;              % Time                                  (1-by-1) 
        g;              % Gravity                               (1-by-1)
        q;              % Joint angles                          (1-by-n) 
        dq;             % Joint velocities                      (1-by-n) 
        ddq;            % Joint accelerations                   (1-by-n) 
        L;              % Length of links                       (1-by-2) 
        Lc;             % Length from proximal joint to C.O.M.  (1-by-2) 
        M;              % Mass of links                         (1-by-2) 
        I;              % Inertia of links, must be diagonal    (2-by-3) 
        
                        % se( 3 ) Matrix for the forward kinematics 
        T01;            % From Frame 0 (WORLD) to Frame 1 (SH)
        T12;            % From Frame 1 (SH)    to Frame 2 (EL)
                        % se( 3 ) Matrix for the forward kinematics 
        T23;            % From Frame 2 (SH2) to Frame 3 (SH3)
        T34;            % From Frame 3 (SH3) to Frame 4 (EL)                        
        T45;            % From Frame 4 (EL)  to Frame 5 (WR1)
        T56;            % From Frame 5 (WR1) to Frame 6 (WR2)
        
        T_arr;          % Array to fill T01, T02
                        % M, C, G Matrix of the upper-limb model 
        M_mat = [];
        C_mat = [];
        G_mat = [];
        
        n_dof;
        
        % Actual values of the mass, coriolis and gravity matrix 
        M_val;
        C_val;
        G_val;
        
        % r is arguments for varargin
        r;
        

        
    end
    
    methods (Static)

        function vs = body2spatial( Rsb, vb )
        % ================================================================             
        % Changing the body frame vector to spatial coordinate
        % ================================================================             
        % [INPUT]
        %    (1) Rsb - Rotation so(3) matrix, which transforms body frame to spatial frame
        %    (2) vb  - Vector described in body (b) frame
        % ================================================================ 
        % [OUTPUT]
        %    (1) vs  - Vector described in spatial (s) frame
        % ================================================================                 
             
             vs = Rsb * vb;
        end 
        
        function [w, v] = se3_2_vec( mat )
        % ================================================================             
        % Changing the se3 matrix to vector format
        % The matrix is in [[w], v; 0,0,0,1] format
        % ================================================================             
        % [INPUT]
        %    (1) mat - 4-by-4 se(3) matrix
        % ================================================================ 
        % [OUTPUT]
        %    (1) v  - Vector form of the se(3) matrix
        % ================================================================    
             w = [ -mat( 2,3 ); mat( 1,3 ); -mat( 1,2 ) ];
             v = mat(1:3, 4);
        end         
        
        function T = se3( q, L, func )
        % ================================================================
        % Returning the 4-by-4 se(3) matrix for forward kinematics
        % Assume that we only consider rotational joints
        % [INPUT]
        %    (1) q (scalar) the amount of rotation
        %    (2) L (3-by-1) The xyz position w.r.t. the frame 
        %    (3) func       rotx, roty, rotz
        % ================================================================
        
            if ~isa( func, 'function_handle' )
                error(  "3rd argument should be function handle of rotx, roty and rotz" )
            end

            name = functions( func ).function;
            if ~( strcmp( name, 'rotx' ) || strcmp( name, 'roty'  ) || strcmp( name, 'rotz' ) ) 
                error(  "3rd argument should be function handle of rotx, roty and rotz" )
            end

            T = [    func( q ),  L; ...
                  zeros( 1,3 ),  1 ];
        
        end
        
        function JT = myTranspose( J )
            % Manual tranpose of the symbolic matrix
            [nR, nC] = size( J );
                
            JT = sym( 'JT', [nC, nR] );
            
            for i = 1 : nR
                for j = 1 : nC
                    JT( j,i ) = J( i,j );
                end
            end

        end
        
        function J = myJacobian( dx1, dx2 )
        % Getting the Jacobian from dx2 to dx1 
        % dx1 = J dx2
            
            J = sym( 'J', [ length( dx1 ), length( dx2 ) ] );

            for i = 1 : length( dx1 )                                           
                for j = 1 : length( dx2 )                                       

                    J( i, j ) = functionalDerivative( dx1( i ), dx2( j ) );

                end
            end

        end
        
        function A = getCoefficient( dx1, dx2 )

            A = sym( 'A', [ length( dx1 ), length( dx2 ) ] );

            for i = 1 : length( dx1 )                                 
                for j = 1 : length( dx2 )                             

                    [ tmpc, tmpt ] = coeffs( dx1( i ), dx2( j ) );    

                    % IF the coefficients (tmpc) corresponding to dq(3) is empty, put zero
                    tmp = tmpc( tmpt == dx2( j ) );
                    if( isempty( tmp ) )
                        A( i, j ) = 0; 
                    else    
                        A( i, j ) = tmp;
                    end

                end
            end
        end   

    end
 

        
    
    methods
        function obj = my2DOFRobotAndRod( varargin ) %
                        
            obj.r = myParser( varargin );
            
            % Defining the symbolic values 
            syms t positive                                                % time               variable - independent variable.
            syms   q1(t)   q2(t)   q3(t)   q4(t)  
            syms  dq1(t)  dq2(t)  dq3(t)  dq4(t)  
            syms ddq1(t) ddq2(t) ddq3(t) ddq4(t)
            
            obj.t   = t;
            obj.q   = [   q1(t),   q2(t)    q3(t),   q4(t) ];
            obj.dq  = [  dq1(t),  dq2(t),  dq3(t),  dq4(t) ];
            obj.ddq = [ ddq1(t), ddq2(t), ddq3(t), ddq4(t) ];
            
            obj.L   = sym( 'L' , [1,2], 'positive' );
            obj.Lc  = sym( 'Lc', [1,2], 'positive' );
            obj.M   = sym( 'M' , [1,2], 'positive' );  
            
            obj.n_dof = 4;
            
            obj.g   = sym('g', 'real' );
            
            syms I1xx I1yy I1zz I2xx I2yy I2zz positive
            obj.I  = [ I1xx, I1yy, I1zz; ...
                       I2xx, I2yy, I2zz] ;
                       
            % The forward kinematics equation in se(3) formulation
            % Basic T matrix, input q (1) and L (3-by-1)  
            obj.T01 = obj.se3( -obj.q( 1 ), zeros( 3, 1 )        , @roty );           
            obj.T12 = obj.se3( -obj.q( 2 ), zeros( 3, 1 )        , @rotx );
            obj.T23 = obj.se3( -obj.q( 3 ), [ 0; 0; -obj.L( 1 ) ], @roty );           
            obj.T34 = obj.se3( -obj.q( 4 ), zeros( 3, 1 )        , @rotx );
            
            
            obj.T_arr = { obj.T01, ...
                          obj.T01 * obj.T12, ...
                          obj.T01 * obj.T12 * obj.T23, ...
                          obj.T01 * obj.T12 * obj.T23 * obj.T34 };
            
            obj.M_mat = obj.getM( );
            obj.C_mat = obj.getC( );
            obj.G_mat = obj.getG( );
            
            obj.M_val     = obj.substitute( obj.M_mat, { 'M', 'L', 'Lc', 'I', 'g' }, obj.r );
            obj.C_val     = obj.substitute( obj.C_mat, { 'M', 'L', 'Lc', 'I', 'g' }, obj.r );
            obj.G_val     = obj.substitute( obj.G_mat, { 'M', 'L', 'Lc', 'I', 'g' }, obj.r );
                        
        end
       
         function eq2 = substitute( obj, eq1, var, values )
            
            eq2 = eq1;
            for i = 1 : length( var )       
                eq2 = subs( eq2, obj.( var{ i } ), values.( var{ i } ) );
            end
            
            
        end
        
        function FK = forwardKinematics( obj, idx, L )
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link, 3 is the 3rd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) q is the relative angle array of the 2DOF robot. 
        %        Can be done in array, which is 2-by-N, N is the time-series data
        % ================================================================ 
        % [OUTPUT]
        %    (1) FK, the position of the given point
        % ================================================================    
        
             %   To neglect the 4th element, 3-by-4 matrix multiplication
             if idx == 1
                FK = [ eye(3), zeros(3,1) ] * obj.T_arr{ 2 } * [ L; 1 ]; 
             elseif idx == 2
                FK = [ eye(3), zeros(3,1) ] * obj.T_arr{ 4 } * [ L; 1 ]; 
             elseif idx == 3
                FK = [ eye(3), zeros(3,1) ] * obj.T_arr{ end } * [ L; 1 ];
             end
             
                
        end
                          
        function pos = calcForwardKinematics( obj, idx, L , qarr )
        % Calculating the xyz position of the given point    
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) qarr is the relative angle array of the 2DOF robot.
        %        size will be 2-by-N
        % ================================================================ 
        % [OUTPUT]
        %    (1) position of the given point, 3-by-N matrix
        % ================================================================              
            pos = obj.forwardKinematics( idx, L );
            pos = obj.substitute( pos, {'L', 'Lc'}, obj.r );
            pos = double( subs( pos, {'q1', 'q2', 'q3', 'q4'}, ...
                            { qarr( 1, : ), qarr( 2, : ), ...
                              qarr( 3, : ), qarr( 4, : ) } ) );
            
        end
        
        
        function M = getM( obj )
        % Calculating the mass matrix of the model 
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================             
        
            % Initializing the mass matrix
            M = 0;  
            
            for i = 1 : length( obj.Lc ) % Iterating along the number of c.o.ms
                
                % Getting the se(3) matrix for the C.O.M.
                tmp_idx = [ 3, 4, 6 ];
                tmp = obj.T_arr{ tmp_idx( i ) } * obj.se3( 0, [ 0; 0; -obj.Lc( i )], @rotz );
                
                % Calculation of body velocity is necessary for getting the generalized mass matrix
                % inv(T) * d(T) = V_b, where V_b is 4-by-4 matrix                
                Vb = simplify( tmp \ diff( tmp, obj.t ) );  
                Vb = subs( Vb, diff( obj.q, obj.t ), obj.dq );             % Simple substitution
                
                % The result of Vb matrix is [[w], v; 0,0,0,1], where w and v are angular and translational velocity, respectively.                      
                [ w, v ] = obj.se3_2_vec( Vb );                            % se(3) to w, v vector transformation
                
                % Calculating the body jacobian 
                J = obj.myJacobian( [v; w], obj.dq );
                
                
                tmp = diag( [ obj.M( i ),   obj.M( i ),   obj.M( i ), ...
                            obj.I( i,1 ), obj.I( i,2 ), obj.I( i,3 )    ]);
            
                M = M + simplify( obj.myTranspose( J ) * tmp * J );
                
            end
               
            obj.M_mat = M;    

        end
        
        function J = calcJacobian( obj, idx, L, qarr )
        % Calculating the xyz position of the given point    
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) qarr is the relative angle array of the 2DOF robot.
        %        size will be 2-by-N
        % ================================================================ 
        % [OUTPUT]
        %    (1) position of the given point, 3-by-N matrix
        % ================================================================              
            tmp = obj.jacobian( idx, L );
            tmp = obj.substitute( tmp, {'L', 'Lc'}, obj.r );
            tmp = double( subs( tmp, {'q1', 'q2', 'q3', 'q4' },  ...
                                     { qarr( 1, : ), qarr( 2, : ), ... 
                                       qarr( 3, : ), qarr( 4, : ) } ) );
            
            % For returning a 3D matrix, 
            nc1 = length( obj.L ); nc2 = length( qarr( 1, : ) );
            
            J = zeros( 3, nc1, nc2 );
            
            for i = 1 : nc2
                J( :, :, i ) = tmp( :, i : nc2 : end );
            end
            
        end
        
        
        function J = jacobian( obj, idx, L )
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) q is the relative angle array of the 2DOF robot.
        %        Can be done in array, which is 2-by-N, N is the time-series data        
        % ================================================================ 
        % [OUTPUT]
        %    (1) J, Jacobian, 3-by-2 matrix
        % ================================================================     
        
            x  = obj.forwardKinematics( idx, L );                          % Getting the position (3) vector from forward kinematics calculation
            
            dx = subs( diff(     x, obj.t ), ...                           % (1st term) Time differentiation of x to get dx/dt
                       diff( obj.q, obj.t ), ...                           % (2nd + 3rd term) Substituting all diff( q,t ) to dq's
                       obj.dq );    
                                                                           
            % Calculating the jacobian from the position                                                                            
            J = obj.myJacobian( dx, obj.dq );

        end
        
        function C = getC( obj )
        % Calculating the coriolis term matrix of the model 
            n = length( obj.q );
            C = sym( 'C', [ n, n ] );

            if isempty( obj.M_mat )
                obj.getM( );        % Fill in the mass matrix if empty. 
            end

            % C matrix is determined by the Christoffel symbols:
            for i = 1 : n
                for j = 1 : n
                    tmp = 0;
                    for k = 1 : n
                        tmp1 =   1 / 2 * functionalDerivative( obj.M_mat( i, j ), obj.q( k ) ) ...
                               + 1 / 2 * functionalDerivative( obj.M_mat( i, k ), obj.q( j ) ) ...
                               - 1 / 2 * functionalDerivative( obj.M_mat( k, j ), obj.q( i ) );
                        tmp1 = tmp1 * obj.dq( k );
                        tmp  = tmp + tmp1;
                    end
                    C( i,j ) = simplify( tmp );
                end
            end        
        
            obj.C_mat = C;    
        end
                
        
        
        function G = getG( obj )
        % Calculating the gravity matrix of the model 
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================                  
            
            G = sym( 'G', [ 1, length( obj.q ) ] );
        
            V = 0;
            
            for i = 1 : length( obj.L )
                pc = obj.forwardKinematics( i, [ 0; 0; -obj.Lc( i )] );
                V  = V + obj.M( i ) * obj.g * pc( end );                   % Getting the z direction
            end
            
            for i = 1 : length( obj.q )
                G( i ) = simplify( functionalDerivative( V, obj.q( i ) ) );
            end
            
            obj.G_mat = G;
        end
        
        function [tau, Y, a] = findManipulatorRegressor( obj )
            % This method is for calculating the  Y matrix and a vector, which will substitute
            % Mq'' + Cq' + G = Ya
            
            M1  = obj.M( 1 );   M2 = obj.M( 2 );   
            L1  = obj.L( 1 );   L2 = obj.L( 2 );   
            Lc1 = obj.Lc( 1 ); Lc2 = obj.Lc( 2 );  
            
            % The possible a vector that might be included in the manipulator equation.
            % Later, it will be trimmed down
            avec = [      obj.I(1,1), obj.I(1,2), obj.I(1,3),  ...
                          obj.I(2,1), obj.I(2,2), obj.I(2,3),  ...
                           M1 * L1^2,  M2 * L1^2, ...
                           M1 * L2^2,  M2 * L2^2, ...
                           M1 * Lc1^2,  M2 * Lc1^2,  ...     
                           M1 * Lc2^2,  M2 * Lc2^2,  ...
                           M1 * Lc1 * L1,  M1 * Lc1 * L2, ...
                           M1 * Lc2 * L1,  M1 * Lc2 * L2, ...
                           M2 * Lc1 * L1,  M2 * Lc1 * L2, ...
                           M2 * Lc2 * L1,  M2 * Lc2 * L2, ...
                           M1 * obj.g * Lc1, M2 * obj.g * Lc1, ...
                           M1 * obj.g * Lc2, M2 * obj.g * Lc2, ...
                           M1 * obj.g * L1, M2 * obj.g * L1, ...
                           M1 * obj.g * L2, M2 * obj.g * L2 ];  
                   
            asym = sym( 'a', [1, length( avec ) ] );  

            ddqr = sym( 'ddqr', [ 1, obj.n_dof ] );
             dqr = sym(  'dqr', [ 1, obj.n_dof ] );
            
            tau = obj.M_mat * obj.myTranspose( ddqr   ) + ...
                  obj.C_mat * obj.myTranspose( dqr    ) + ...
                              obj.myTranspose( obj.G_mat  );
             
            tau_sub = sym( 'tau_sub', [ obj.n_dof,1] );

            for i = 1 : obj.n_dof

                tmp = tau( i );
                
                for j = 1 : length( avec ) 

                    tmp = subs( expand( tmp ), avec( j ), asym( j ) );

                end
                
                tau_sub( i ) = tmp;
            end       
                   
         
            % Y is simply the jacobian of tau with respect to a
            Y = obj.getCoefficient( tau_sub, asym );
            % Clean up the columns filled with 0, 
            % Get the index of it
            
            idx = ~all( Y == 0 );
            Y   =   simplify( Y( :,idx ) );
            a   =   avec(  idx );
            
            
            % After we tease out the Y Matrix, find the columns which are same. 
            i = 1; 
            while i ~= length( a )
                ismatch = false;
                for j = i + 1 : length( a )
                    if isequal( Y( :, i ), Y( :, j ) )
                        % If column is equal, add the a values and erase the part
                        a( i ) = a( i ) + a( j );
                        a( j ) = []; Y( :, j ) = [];
                        ismatch = true;
                        break
                    end
                end
                
                if ~ismatch
                   i = i + 1; 
                end
                
            end
            
        end

        
        
    end

        
end


