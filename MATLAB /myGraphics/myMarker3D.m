classdef myMarker3D < myGraphics
% % =============================================================== %
%   [DESCRIPTION]
%       myMarker3D class for defining a single marker in the plot
%
%
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     31-Jan-2021
% % =============================================================== %
    
    properties ( SetAccess = private )

    end


    properties ( SetAccess = public )
        
        % [Properties]
        %   - The variables should match the name with the "scatter" function attributes 
        name  = "g" + num2str( randi ( 2^20 ) );                           % Randomizing the name of this marker
        hfunc = @scatter3;                                                 % The function handle for this graphic object.        
        
        % x,y position data of the marker
        XData
        YData
        ZData
                
        % Graphic attributes
        LineWidth       = 0.5;
        Marker          = 'o';
        MarkerFaceColor = [1,      1,      1 ];
        MarkerEdgeColor = [0, 0.4470, 0.7410 ];
        MarkerFaceAlpha = 1;
        MarkerEdgeAlpha = 1;  

    end
    
    methods

        function obj = myMarker3D( varargin )
            % Construct an instance of the marker class
            % [Inputs Arguments]
            %       varargin: The varargin should always get the value in 'key', 'val' pairs./
            %        example: 'name', '123', 'XData', [1,3], 'YData', [3,2]
            
            obj.setAttr( varargin{ : } )                                % The initialize function is defined under "myGraphics.m" file.
            
        end
 
    end
end
