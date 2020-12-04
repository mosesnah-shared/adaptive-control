classdef my4DOFRobot < my2DOFRobot
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an 4-DOF planar robot 
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     18-October-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
    end
        
    properties ( SetAccess = public )

        
    end
    
    methods (Static)

  
    end            
 

    methods
        function obj = my4DOFRobot( varargin ) %
                        
            obj.nDOF  = 4;                
            obj.idx_limb = [ 3, 4 ];                   
        end

        function [M, C, G] = deriveManipulatorEquation( obj )
            % Derive the manipulator equation:
            % tau = Mq'' + Cq' + G
            
            % Defining the se(3) matrix for each coordinate transformation        
            T01 = obj.se3( -obj.q( 1 ), zeros( 3, 1 )        , @roty );           
            T12 = obj.se3( -obj.q( 2 ), zeros( 3, 1 )        , @rotx );
            T23 = obj.se3(  obj.q( 3 ), zeros( 3, 1 )        , @rotz );
            T34 = obj.se3( -obj.q( 4 ), [ 0; 0; -obj.L( 1 ) ] , @roty );
            
            % [WARNING!!!]
            % We need to fill in the T_arr before we conduct .getM, .getC and .getG methods
            obj.T_arr = { T01, T01 * T12, T01 * T12 * T23,  T01 * T12 * T23 * T34 };
            
            M = obj.getM( );
            C = obj.getC( );
            G = obj.getG( );

        end
       
        

    end

        
end


