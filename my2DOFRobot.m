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
        
        L;                                                                 % (1-by-2) Length of each link
        Lc;                                                                % (1-by-2) Length measured from the proximal joint to C.O.M. (Center of Mass)
        M;                                                                 % (1-by-2) Mass of each link
        I;                                                                 % (2-by-6) Inertia matrix of each link, since it is symmetric, we need 6 parameters
                                                                           %          In order, Ixx, Iyy, Izz, Ixy, Ixz, Iyz
                                                                           
        % Symbolic values
        sym_q;
        sym_dq;
        sym_ddq;
        sym_L;                                                                    
        sym_Lc;
        sym_M;
        sym_I;
        
        % se( 3 ) Matrix for the forward kinematics 
        T01;
        T02;
        
         func_FK;       % function handle array of forward kinematics
        func_FK1;       % The forward kinematics equation for link1
        func_FK2;       % The forward kinematics equation for link2
        
    end
    
    methods (Static)
                
%         function p = polar2xy( r, phi )
%              p = [ r * cos( phi ), r * sin( phi ) ]; 
%         end 
    end
        
    
    methods
        function obj = my2DOFRobot( varargin )
            
            r = myParser( varargin );               
            
            % [TODO] Setting a one2one correspondence between variable and its name for single-line conversion. 
            obj.L  = r.length;
            obj.Lc = r.lengthCOM;
            obj.M  = r.mass;
            obj.I  = r.inertia;
            
            syms t positive                                                % time               variable - independent variable.
            syms   q1(t)   q2(t)                                           % joint angle        variable -   dependent variable.
            syms  dq1(t)  dq2(t)                                           % joint velocity     variable -   dependent variable.
            syms ddq1(t) ddq2(t)                                           % joint acceelration variable -   dependent variable.            
            
            obj.sym_q   = [   q1(t),   q2(t) ];
            obj.sym_dq  = [  dq1(t),  dq2(t) ];
            obj.sym_ddq = [ ddq1(t), ddq2(t) ];
            
            obj.sym_L   = sym( 'L' ,[1,2] );
            obj.sym_Lc  = sym( 'Lc',[1,2] );
            obj.sym_M   = sym( 'M' ,[1,2] );
               
            syms I1xx I1yy I1zz I1xy I1xz I1yz I2xx I2yy I2zz I2xy I2xz I2yz
            obj.sym_I  = [ I1xx, I1yy, I1zz, I1xy, I1xz, I1yz; ...
                           I2xx, I2yy, I2zz, I2xy, I2xz, I2yz ];
                       
            % The forward kinematics equation in se(3) formulation
            obj.T01 = @( q, L ) [ rotz( q( 1 ) ), zeros(3,1); zeros(1,3), 1 ];
            obj.T02 = @( q, L ) obj.T01( q, L ) * [ rotz, L(2);0;0 ; zeros(1,3), 1 ];
                       
                       
            % The forward kinematics equation for link 1
%             obj.func_FK1 = @( q, L ) rotz( q( 1 ) ) * [ L; 0; 0 ];
%             obj.func_FK2 = @( q, L ) obj.func_FK1( q, obj.sym_L( 1 ) ) + rotz( sum( q ) ) * [ L; 0; 0 ];
%             obj.func_FK  = { obj.func_FK1, obj.func_FK2 };
        end

        function FK = forwardKinematics( obj, idx, L, q )
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
        
            FK = obj.func_FK{ idx }( q, L );
                
        end
            
        
        function J = jacobian( obj, idx, L, q )
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
        
            pos = obj.forwardKinematics( idx, L, q );
            
            % Calculating the jacobian from the position 
            J = jacobian( pos, obj.sym_q ); % This is NOT the method from this class, but the function defined in MATLAB 
        
        end
        
        function eq = getEOM( obj, type )
        % ================================================================             
        % [INPUT]            
        %   (1) type, "num" or "sym".
        %       If num, then the equation will be in numeric value with the corresponding L, Lc, M and I values
        %       If sum, then the equation will be in symbolic expressions/
        % ================================================================             
        % [OUTPUT]
        %    (1) Equation/function handle of the robot        
        % ================================================================ 
        
        
        end
        
        
    end

        
end


