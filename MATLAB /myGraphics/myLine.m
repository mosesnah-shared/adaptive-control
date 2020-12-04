classdef myLine < handle
% % =============================================================== %
%   [DESCRIPTION]
%
%       myLine class for defining a 2D/3D Line Object for the plot
%
%   [CONSTRUCTOR]
%      myLine( x, y, z, varargin )
%
%      (1) x,y,z: The 1-by-N row vectors, x, y and z position where the vector is pointing w.r.t. the origin.
%      
%      (2) varargin: The properties of the line object.
%               (a) lineColor
%               (b) lineWidth
%               (c) lineStyle
%       
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     08-June-2020    
% % =============================================================== %
    
    properties ( SetAccess = public )
        name
        x
        y
        z
        lineColor
        lineWidth
        lineStyle
    end
    
    methods

        function obj = myLine( x, y, z, varargin )
            obj.x = x; obj.y = y; obj.z = z;            
            
            r = myParser( varargin );                
            
            obj.lineWidth = r.lineWidth;
            obj.lineColor = r.lineColor;
            obj.lineStyle = r.lineStyle;
                        
        end
        

    end
end

