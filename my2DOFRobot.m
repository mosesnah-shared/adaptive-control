classdef my2DOFRobot < handle
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an 2-DOF planar robot 
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     18-October-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
    end
        
    properties ( SetAccess = public )
        
%         L;                                                                 % (1-by-2) Length of each link
%         Lc;                                                                % (1-by-2) Length measured from the proximal joint to C.O.M. (Center of Mass)
%         M;                                                                 % (1-by-2) Mass of each link
%         I;                                                                 % (2-by-6) Inertia matrix of each link, since it is symmetric, we need 6 parameters
%                                                                            %          In order, Ixx, Iyy, Izz, Ixy, Ixz, Iyz
%                                                                            
        % Symbolic values
        t; 
        q;
        dq;
        ddq;
        L;                                                                    
        Lc;
        M;
        I;
        
        % se( 3 ) Matrix for the forward kinematics 
        T;
        T01;
        T02;
        T_arr;
        
        % M, C, G Matrix of the upper-limb model 
        M_mat = [];
        C_mat = [];
        G_mat = [];
        
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

    end
        
    
    methods
        function obj = my2DOFRobot( ) %varargin )
                        

            syms t positive                                                % time               variable - independent variable.
            syms   q1(t)   q2(t)                                           % joint angle        variable -   dependent variable.
            syms  dq1(t)  dq2(t)                                           % joint velocity     variable -   dependent variable.
            syms ddq1(t) ddq2(t)                                           % joint acceelration variable -   dependent variable.            
            
            obj.t   = t;
            obj.q   = [   q1(t),   q2(t) ];
            obj.dq  = [  dq1(t),  dq2(t) ];
            obj.ddq = [ ddq1(t), ddq2(t) ];
            
            obj.L   = sym( 'L' ,[1,2], 'positive' );
            obj.Lc  = sym( 'Lc',[1,2], 'positive' );
            obj.M   = sym( 'M' ,[1,2], 'positive' );       
            
            syms I1xx I1yy I1zz I2xx I2yy I2zz positive
            obj.I  = [ I1xx, I1yy, I1zz; ...
                       I2xx, I2yy, I2zz];
                       
            % The forward kinematics equation in se(3) formulation
            % Basic T matrix, input q (1) and L (3-by-1)  
            obj.T   = @( q, L ) [ rotz( q ),  L; ...
                               zeros( 1,3 ),  1 ];
            
            obj.T01 = @( q ) obj.T( q( 1 ), zeros( 3, 1 ) );               % Forward kinematics for the first link
            obj.T02 = @( q ) obj.T( q( 1 ), zeros( 3, 1 ) ) * obj.T( q( 2 ), [ obj.L( 1 ); 0; 0 ] ) ;
                       
            obj.T_arr = { obj.T01, obj.T02 };

        end

        function FK = forwardKinematics( obj, idx, L )
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) q is the relative angle array of the 2DOF robot. 
        %        Can be done in array, which is 2-by-N, N is the time-series data
        % ================================================================ 
        % [OUTPUT]
        %    (1) FK, the position of the given point
        % ================================================================    
        
             %   To neglect the 4th element, 3-by-4 matrix multiplication
             FK = [ eye(3), zeros(3,1) ] * obj.T_arr{ idx }( obj.q ) * [ L; 1 ];
                
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
            
            dx = subs( diff( x, obj.t ), ...                               % (1st term) Time differentiation of x to get dx/dt
                       diff( obj.q, obj.t ), ...                           % (2nd + 3rd term) Substituting all diff( q,t ) to dq's
                       obj.dq );    
                                                                           
            % Calculating the jacobian from the position                                                                            
            J = myJacobian( dx, obj.dq );


        end
        
        function C = getC( obj )
        % Calculating the coriolis term matrix of the model 
        % C matrix is determined by the Christoffel symbols:
        n = length( obj.q );
        C = sym( 'C', [ 2, 2 ] );

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
                    C(i,j) = tmp;
                end
            end        
        
            C = simplify( C ); 
            obj.C_mat = C;    
        end
        
        function G = getG( obj )
        % Calculating the gravity matrix of the model 
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================                  
            
            % Calculating the potential energy V is easy
            syms g real
            pC1 = obj.forwardKinematics( 1, [ obj.Lc( 1 ); 0; 0] );        % Position of C.O.M. Link 1
            pC2 = obj.forwardKinematics( 2, [ obj.Lc( 2 ); 0; 0] );        % Position of C.O.M. Link 2                      
            
            V = obj.M( 1 ) * g * pC1( 2 ) + obj.M( 2 ) * g * pC2( 2 );
            
            G = sym( 'G', [ 1, length( obj.q) ]);
            for i = 1 : length( obj.q )
                G(i) = functionalDerivative( V, obj.q( i ) );
            end
            G = simplify( G );
            obj.G_mat = G;
        end
        
        
        function M = getM( obj )
        % Calculating the mass matrix of the model 
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================             
        
            % Using a Lagrangian-based approach for deriving the generalized mass matrix
            tmp1 = obj.T01( obj.q ) * obj.T( 0, [ obj.Lc(1); 0; 0] );      % se(3) matrix for the C.O.M. of the 1st link
            tmp2 = obj.T02( obj.q ) * obj.T( 0, [ obj.Lc(2); 0; 0] );      % se(3) matrix for the C.O.M. of the 2nd link

            % Calculation of body velocity is necessary for getting the generalized mass matrix
            % inv(T) * d(T) = V_b, where V_b is 4-by-4 matrix
            Vb_c1 = simplify( inv( tmp1 ) * diff( tmp1, obj.t ) );             
            Vb_c2 = simplify( inv( tmp2 ) * diff( tmp2, obj.t ) );
                    
            % Simple Substitution
            Vb_c1 = subs( Vb_c1, diff( obj.q, obj.t ), obj.dq );           
            Vb_c2 = subs( Vb_c2, diff( obj.q, obj.t ), obj.dq );            
            
            % The result of Vb matrix is [[w], v; 0,0,0,1], where w and v are angular and translational velocity, respectively.                      
            [wb1, vb1] = obj.se3_2_vec( Vb_c1 );                               % se(3) to w, v vector transformation
            [wb2, vb2] = obj.se3_2_vec( Vb_c2 );                               % se(3) to w, v vector transformation
            
           
            M1 = diag( [ obj.M(1), obj.M(1), obj.M(1), obj.I(1,1), obj.I(1,2), obj.I(1,3) ]);
            M2 = diag( [ obj.M(2), obj.M(2), obj.M(2), obj.I(2,1), obj.I(2,2), obj.I(2,3) ]);

            JBody1 = myBodyJacobian( wb1, vb1, obj.dq );
            JBody2 = myBodyJacobian( wb2, vb2, obj.dq );

            % The reason for the transpose is due to the 'conjugate' of matlab
            % If you simply do ' for tranpsose, they give us the conjugated value. 
            M = simplify( myTranspose( JBody1 ) * M1 * JBody1 + ...
                          myTranspose( JBody2 ) * M2 * JBody2 );

            obj.M_mat = M;            
        end

        
        function out = substitute( obj, eq, varargin )
            % Substitute the variables with actual values, that might become handy for ode calculation
            r = myParser( varargin );  
            
            tmp = subs(  eq, { 'M1',  'M2'}, { r.mass      } );
            tmp = subs( tmp, { 'L1',  'L2'}, { r.length    } );
            tmp = subs( tmp, {'Lc1', 'Lc2'}, { r.lengthCOM } );
            tmp = subs( tmp, {'I1zz', 'I2zz'}, { r.inertia(1,3), r.inertia(2,3) } );
            
            
            tmp = subs( tmp, 'g', r.gravity );
            
            out = tmp;
            
        end
        
        function [pSH, pEL, pEE] = FKcalc( obj, qarr )
        % Calculating the actual q array value     
            pSH = zeros( 3, size( qarr, 2 ) );    
            
            
            obj.forwardKinematics( 1, [ obj.L( 1 ); 0; 0] )
            obj.forwardKinematics( 2, [ obj.L( 2 ); 0; 0] )
        end
    end

        
end


