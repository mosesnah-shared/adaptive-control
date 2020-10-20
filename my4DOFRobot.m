classdef my4DOFRobot < my2DOFRobot
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
                        % se( 3 ) Matrix for the forward kinematics 
        T23;            % From Frame 2 (SH2) to Frame 3 (SH3)
        T34;            % From Frame 3 (SH3) to Frame 4 (EL)
        
    end
    
    methods (Static)

       
    end
        
    
    methods
        function obj = my4DOFRobot( varargin ) %
                        
            obj.r = myParser( varargin );
            
            % Defining the symbolic values 
            syms t positive                                                % time               variable - independent variable.
            syms   q1(t)   q2(t)   q3(t)   q4(t)                           % joint angle        variable -   dependent variable.
            syms  dq1(t)  dq2(t)  dq3(t)  dq4(t)                           % joint velocity     variable -   dependent variable.
            syms ddq1(t) ddq2(t) ddq3(t) ddq4(t)                           % joint acceelration variable -   dependent variable.            
            
            obj.t   = t;
            obj.q   = [   q1(t),   q2(t)    q3(t),   q4(t) ];
            obj.dq  = [  dq1(t),  dq2(t),  dq3(t),  dq4(t) ];
            obj.ddq = [ ddq1(t), ddq2(t), ddq3(t), ddq4(t) ];
            
            obj.L   = sym( 'L' ,[1,2], 'positive' );
            obj.Lc  = sym( 'Lc',[1,2], 'positive' );
            obj.M   = sym( 'M' ,[1,2], 'positive' );  
            
            obj.g   = sym('g', 'real' );
            
            syms I1xx I1yy I1zz I2xx I2yy I2zz positive
            obj.I  = [ I1xx, I1yy, I1zz; ...
                       I2xx, I2yy, I2zz];
                       
            % The forward kinematics equation in se(3) formulation
            % Basic T matrix, input q (1) and L (3-by-1)  
            obj.T01 = obj.se3( -obj.q( 1 ), zeros( 3, 1 )        , @roty );           
            obj.T12 = obj.se3( -obj.q( 2 ), zeros( 3, 1 )        , @rotx );
            obj.T23 = obj.se3(  obj.q( 3 ), zeros( 3, 1 )        , @rotz );
            obj.T34 = obj.se3( -obj.q( 4 ), [ 0; 0; -obj.L( 1 ) ] , @roty );
            
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
                tmp = obj.T_arr{ end - 2 + i } * obj.se3( 0, [ 0; 0; -obj.Lc( i )], @rotz );
                
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


        
        
    end

        
end


