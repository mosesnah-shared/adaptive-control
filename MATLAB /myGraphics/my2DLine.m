classdef my2DLine < handle
% % =============================================================== %
%   [DESCRIPTION]
%
%       my2DLine class for defining a 2D Plotting Line Object.
%       This Graphical Object is "Static", which means the line is not updated w.r.t. time step.
%
%   [CONSTRUCTOR]
%      my2DLine( x, y, varargin )
%
%      (1) x,y: The 1-by-N row vectors, x, y and z position where the vector is pointing w.r.t. the origin.
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
        lineColor
        lineWidth
        lineStyle
    end
    
    methods

        function obj = my2DLine( x, y, varargin )
            obj.x = x; obj.y = y; 
            
            r = myParser( varargin );                
            
            obj.lineWidth = r.lineWidth;
            obj.lineColor = r.lineColor;
            obj.lineStyle = r.lineStyle;
                        
        end
        

    end
end

