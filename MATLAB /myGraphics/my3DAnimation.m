classdef my3DAnimation < handle
%
%  my3DAnimation class for setting up the 3D animation with the input data.
% 
% % =============================================================== %
%   [DESCRIPTION]
%
%
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     08-June-2020    
% % =============================================================== %


% % =============================================================== %
%   [REFERENCES]
%   my3DAnimation class for setting up the 3D animation with the input data.
% 
% % =============================================================== %

% % =============================================================== %
% [START] 

    properties ( SetAccess = private )
        
        % ================================================================= % 
        % [Figure and Axes Configuration]
        % suffix m  stands for main
        % suffix s1 stands for side plot #1 (Lower Right)
        % suffix s2 stands for side plot #2 (Upper Right)
        
                                           % [Configuration #1] A single big plot
        pos1m  = [0.08 0.10 0.84 0.80];    %   Position/size for the main plot before adding plot 
        
                                           % [Configuration #2] A big plot on the left, two plots on the upper/lower right.
        pos2   = [0.08 0.08 0.42 0.82; ... %   Position/size for the main plot - 3D real time plot
                  0.58 0.08 0.40 0.37; ... %   Position/size for the under-sub-plot, drawn in the lower section
                  0.58 0.55 0.40 0.40]     %   Position/size for the above-sub-plot, drawn in the upper section.

    end
    
    properties ( SetAccess = public )
        % ================================================================ %
        % [Syntax]
        % (1) Graphic Handle
        %     - h Stands for handle
        %     - F, A, L, M, E Stands for Figure, Axes, Line, Markers, Ellipse Handle, respectively.
        %     - m, s1, s2 stands for main, side1, side2, respectively.
        %     - s1Z, Z suffix stands for "Zoomed" Plot, which is special 
        % (2) Position Information 
        %     - p prefix stands for XYZ position information.
        %     - If pX, then position of X.
        % (3) Indexing Information 
        %     - The indexing information is important for "connect" method 
        %     - "idx" prefix stands for indexing
        % ================================================================ %
        
        % ================================================================ %
        % [START OF DEFINITION]
        
        tVec; tStep;            % Time vector and step of the simulation.
        simStep = 1;            % The step number of the simulation.
        
        vidRate = 0;            % Default as zero
        
                                % [The handle of the Figure and Title]
        hFigure = []            % The handle of the whole figure
        hTitle  = []            % The  title of the whole figure.

                                % [The handle of the axes]
          % The list of axes, 1,2,3 is ordered as main, side1, side2
        hAxes     = {     gobjects(0),     gobjects(0),       gobjects(0) };
          
          
        hMarkers   = {     gobjects(0),     gobjects(0),       gobjects(0) };
        markers    = {  myMarker.empty,  myMarker.empty,    myMarker.empty };        

        hLines     = {     gobjects(0),     gobjects(0),       gobjects(0) };
        lines      = {    myLine.empty,    myLine.empty,      myLine.empty };                                     
        
        hpLines    = {     gobjects(0),     gobjects(0),       gobjects(0) }; % Lines for "Plots", which are static.
        pLines     = {  my2DLine.empty,  my2DLine.empty,    my2DLine.empty }; % Lines for "Plots", which are static.         
        
        hEllipses  = {     gobjects(0),     gobjects(0),       gobjects(0) };
        ellipses   = { myEllipse.empty, myEllipse.empty,  myEllipse.empty  };
        
        hArrows    = {     gobjects(0),     gobjects(0),       gobjects(0) };
        arrows     = {   myArrow.empty,   myArrow.empty,    myArrow.empty  };
        
        isZoomed   = [           false,          false,               false]; % Check whether the plot is a zoomed-plot or not
        zoomIdx; zoomSize;
    end
    
    methods
                
        function obj = my3DAnimation( tStep, markers, varargin )
            %[CONSTRUCTOR #1] Construct an instance of this class
            %   (1) tVec [sec]
            %       -  The time vector of the simulation
            %   (2) markers [marker array] 
            %       -  The class array of "myMarker" class. 
            
            % [Quick Sanity Check] Check the size between the data and time vec    
            
            for m = markers
                if ( m.N ~= markers( 1 ).N  )
                    error( "Wrong size [Name: %s] %d != %d " ,m.name, m.N, marker( 1 ).N )
                end
            end
                        
            % If the size of the data are all the same.
            % Setting the default time step and its corresponding time vector.
            obj.tStep = tStep;
            obj.tVec  = tStep * (0 : m.N-1);   % Taking out 1 to match the size of data        
            
            % Setting the default figure and Axes for the plot
            obj.hFigure     = figure();
            obj.hAxes{ 1 }  = subplot( 'Position', obj.pos1m, 'parent', obj.hFigure );    
            obj.hTitle      = title( obj.hAxes{ 1 }, sprintf( '[Time] %5.3f (s)', obj.tVec( 1 ) ) );
            hold( obj.hAxes{ 1 },'on' ); axis( obj.hAxes{ 1 } , 'equal' )             

            tmpN = length( markers );                                      % Getting the length of the markers
            % Plotting the Markers 

            for i = 1 : tmpN
                
                m = markers( i );
                obj.addGraphicObject( 1, m )
                              
            end
                        
        end
        
        function adjustFigures( obj, idx )
             %adjustFigures: adjusting the positioning of figures for multiple plots
             % This function is more of a internal function.
             % [INPUT]
             %    (1) idx, 1 (Main), 2 (Lower-right), 3 (Upper-right)
             
            if  ( idx == 2 ) && ( isempty( obj.hAxes{ 2 } ) )
                set( obj.hAxes{ 1 }, 'Position', obj.pos2( 1,: ) ); 
                
                obj.hAxes{ 2 } = subplot( 'Position', obj.pos2( 2, : ), 'parent', obj.hFigure );
                hold( obj.hAxes{ 2 },    'on' );  %axis( obj.hAxes{ 2 }, 'equal' ); 
            end   
            
            if  ( idx == 3 ) && ( isempty( obj.hAxes{ 3 } ) )
                set( obj.hAxes{ 1 }, 'Position', obj.pos2( 1,: ) ); 
                
                obj.hAxes{ 3 } = subplot( 'Position', obj.pos2( 3, : ), 'parent', obj.hFigure );
                hold( obj.hAxes{ 3 },    'on' );  axis( obj.hAxes{ 3 }, 'equal' ); 
            end   
            
            
        end
        
        function addGraphicObject( obj, idx, myGraphics, varargin )
             %addGraphicObject: adding a single graphical object to the plot
             % [INPUT]
             %   (1) idx [integer]
             %       -  1 (Main), 2 (Lower-right), 3 (Upper-right)
             %   (2) myGraphics [marker array] 
             %       -  The graphics class that we are aimed to add
             %       -  Type of Graphic Objects
             %          (a) Ellipse
             %          (b) Vector
             %          (c) myArrow             
             %    (3) varagin
             %     - [TO BE ADDED]             

            obj.adjustFigures( idx )         % Change the configuration of the plots if side plots are not drawn
            h2Draw = obj.hAxes{ idx };       % Getting the axes to do the plot
            
            if     isa( myGraphics , 'myArrow' )
                
                for g = myGraphics
                   arrow = quiver3( g.orig( 1,1 ), g.orig( 2,1 ), g.orig( 3,1 ), ...
                                           g.x(1),        g.y(1),        g.z(1), ...
                                                               'parent', h2Draw, ...
                                                      'linewidth', g.arrowWidth, ...
                                                          'color', g.arrowColor, ...
                                                    'MaxheadSize', g.arrowHeadSize );
                 
                     obj.hArrows{ idx }( end + 1 ) = arrow;       
                      obj.arrows{ idx }( end + 1 ) = g;
                end    
                      
            elseif isa( myGraphics , 'myEllipse' )
                
                for g = myGraphics                      
                      ellipse = mesh( g.xmesh(:,:,1), g.ymesh(:,:,1), g.zmesh(:,:,1), ...
                                       'parent', h2Draw, 'facealpha', g.faceAlpha );

                     obj.hEllipses{ idx }( end + 1 ) = ellipse;       
                      obj.ellipses{ idx }( end + 1 ) = g;                               
                end  
                
            elseif isa( myGraphics, 'myMarker' )
                
                for g = myGraphics                      

                    % If size is correct, add the markers to the plot
                        marker = scatter3( g.xdata( 1 ), g.ydata( 1 ), g.zdata( 1 ), g.markerSize * 8 , ...
                                                                'parent', h2Draw,  ...
                                                        'Marker',  g.markerStyle,  ...
                                                     'LineWidth',  g.markerSize/3, ...   
                                               'MarkerEdgeColor',  g.markerColor,  ...
                                               'MarkerEdgeAlpha',  g.markerAlpha,  ...
                                               'MarkerFaceAlpha',  g.markerAlpha,  ...
                                               'MarkerFaceColor',  [1,1,1]  ); 

                        obj.hMarkers{ idx }( end + 1 ) = marker;
                         obj.markers{ idx }( end + 1 ) = g; 
                end
                
            elseif isa( myGraphics, 'myLine' )
                
                 for g = myGraphics                      

                    line = plot3( g.x(:,1), g.y(:,1), g.z(:,1), ...
                                              'parent', h2Draw, ...
                                      'lineWidth', g.lineWidth, ...
                                          'color', g.lineColor, ...                           
                                      'lineStyle', g.lineStyle );

                        obj.hLines{ idx }( end + 1 ) = line;
                         obj.lines{ idx }( end + 1 ) = g; 
                 end    

            elseif isa( myGraphics, 'my2DLine' )
                
                 for g = myGraphics                      

                    line = plot( g.x, g.y, 'parent', h2Draw, ...
                                   'lineWidth', g.lineWidth, ...
                                       'color', g.lineColor, ...                           
                                   'lineStyle', g.lineStyle );
 
                        obj.hpLines{ idx }( end + 1 ) = line;
                         obj.pLines{ idx }( end + 1 ) = g; 
                 end                     
                 
            end
            
        end
        
        function connectMarkers( obj, idx, whichMarkers, varargin )
            %connect: connecting markers with lines in the main plot
            % =============================================================== %
            % [INPUTS]
            %   (1) idx [integer]
            %       -  The plot that are aimed to 
            %   (1) whichMarkers (string/index) List
            %       - the index or the name of the markers that are aimed to be connected
            %   (2) varargin
            %       - [TO BE ADDED]              
            
            % Temporary Saving the list of names of the markers for the connect
            obj.adjustFigures( idx )            % Change the configuration of the plots if side plots are not drawn
            
            N = length( whichMarkers );
            
            % Allocating the x,y and z markers
            tmpx = zeros( N, length( obj.tVec) ); 
            tmpy = tmpx; 
            tmpz = tmpx;
            
            if     isstring( whichMarkers )     % If markers info. are given as strings
                
                tmp = [ obj.markers{ idx }.name ];

                for i = 1 : N
                    tmpi = find( strcmp( tmp, whichMarkers( i ) ) );       % Getting the index of the marker which matches the string name
                    
                    tmpx( i,: ) = obj.markers{ idx }( tmpi ).xdata;
                    tmpy( i,: ) = obj.markers{ idx }( tmpi ).ydata;
                    tmpz( i,: ) = obj.markers{ idx }( tmpi ).zdata;                    
                    
                end
                
            elseif isnumeric( whichMarkers )                               % If markers info. are given as integer array 
                     
                for i = 1 : N
                    
                    tmpx( i,: ) = obj.markers{ idx }( whichMarkers( i ) ).xdata;
                    tmpy( i,: ) = obj.markers{ idx }( whichMarkers( i ) ).ydata;
                    tmpz( i,: ) = obj.markers{ idx }( whichMarkers( i ) ).zdata;                    
                    
                end
                
            else
                error( "Wrong input, argument should be array of strings or integers" ) 
            end
            

            r = myParser( varargin );   

            line = myLine( tmpx, tmpy, tmpz, 'lineStyle', r.lineStyle, ...
                                             'lineColor', r.lineColor, ...
                                             'lineWidth', r.lineWidth );
            
            obj.addGraphicObject( idx, line );
                                         
        end        

        function run( obj, vidRate, duration, isVidRecord, videoName )
            %run: running the a whole animation and saving the video if defined.
            % [INPUTS]
            %   (1) vidRate 
            %       - video Rate of the animation.
            %       - if vidRate is 0.5, then the video is two times slower. 
            %       - if vidRate is 1.0, then the video is at real time speed.
            %       - if vidRate is 2.0, then the video is two times faster. 
            %   (2) duration
            %       - Duration of the animation        
            %
            %   (3) isVidRecord ( boolean )
            %       - true or false, if false, simply showing the animation.
            %
            %   (4) videoName (string)
            %       - name of the video for the recording.

            if duration >= max( obj.tVec )
               duration = max( obj.tVec ); 
            end
                
            N    = round( duration/ obj.tStep );
            obj.vidRate = vidRate;     
            step = round( vidRate * ( 1 / obj.tStep / 60 ) );              % Setting 60 fps - 1 second as default!

            if step == 0                                                   % In case the step is too small, then set the simStep as 1
               step = 1; 
            end
            
            if isVidRecord                                                 % If video record is ON
                
                if vidRate >=1 
                    fps = 30;
                else
                    fps = 30 * vidRate; 
                end
                writerObj = VideoWriter( videoName, 'MPEG-4' );            % Saving the video as "videoName" 
                writerObj.FrameRate = fps;                                 % How many frames per second.
                open( writerObj );                                         % Opening the video write file.

            end    
            


            for i = 1 : step : N
                
                obj.goto( i )                                           % Run a single step of the simulation

                if isVidRecord                                             % If videoRecord is ON
                    frame = getframe( obj.hFigure );                       % Get the current frame of the figure
                    writeVideo( writerObj, frame );                        % Writing it to the mp4 file/
                else                                                       % If videoRecord is OFF
                    drawnow                                                % Just simply show at Figure
                end

            end   

            if isVidRecord
                close( writerObj );
            end         

        end
        
        function goto( obj, idx )
            % Go to the step of the following idx. 
            set( obj.hTitle,'String', sprintf( '[Time] %5.3f (s)  x%2.1f', obj.tVec( idx ),  obj.vidRate  ) )
        
                  % Update Markers
            for i = 1 : length( obj.hMarkers )
               
                if isempty( obj.hMarkers{ i } )
                    continue     % Go to next iteration
                end
                
                for j = 1 : length( obj.hMarkers{ i } )
                    
                    set( obj.hMarkers{ i }( j ), 'XData', obj.markers{ i }( j ).xdata( idx ), ...
                                                 'YData', obj.markers{ i }( j ).ydata( idx ), ...
                                                 'ZData', obj.markers{ i }( j ).zdata( idx )   )
                    
                end
                 
            end
            
            % Update Ellipse            
            for i = 1 : length( obj.hEllipses )
               
                if isempty( obj.hEllipses{ i } )
                    continue     % Go to next iteration
                end
                
                for j = 1 : length( obj.hEllipses{ i } )
                    set( obj.hEllipses{ i }( j ), 'XData', obj.ellipses{ i }( j ).xmesh( :, :, idx  ), ...
                                                  'YData', obj.ellipses{ i }( j ).ymesh( :, :, idx  ), ...
                                                  'ZData', obj.ellipses{ i }( j ).zmesh( :, :, idx  ) )
                    
                end
                 
            end            
            
            % Update Lines            
            for i = 1 : length( obj.hLines )
               
                if isempty( obj.hLines{ i } )
                    continue     % Go to next iteration
                end
                
                for j = 1 : length( obj.hLines{ i } )
                    set( obj.hLines{ i }( j ), 'XData', obj.lines{ i }( j ).x( :, idx ), ...
                                               'YData', obj.lines{ i }( j ).y( :, idx ), ...
                                               'ZData', obj.lines{ i }( j ).z( :, idx )  )
                    
                end
                 
            end
            
            % Update Arrows            
            for i = 1 : length( obj.hArrows )
               
                if isempty( obj.hArrows{ i } )
                    continue     % Go to next iteration
                end
                
                for j = 1 : length( obj.hArrows{ i } )
                    set( obj.hArrows{ i }( j ), 'XData', obj.arrows{ i }( j ).orig( 1, idx ), ...
                                                'YData', obj.arrows{ i }( j ).orig( 2, idx ), ...
                                                'ZData', obj.arrows{ i }( j ).orig( 3, idx ), ...
                                                'UData', obj.arrows{ i }( j ).x( idx ), ...
                                                'VData', obj.arrows{ i }( j ).y( idx ), ...
                                                'WData', obj.arrows{ i }( j ).z( idx )   )
                    
                end
                 
            end
            
            % Update the zoomed-in view's xlim, ylim and zlim
            
            iidx = find( obj.isZoomed );  % Find the index that should be changed. 
            if iidx ~= 0
                set( obj.hAxes{ iidx },  'XLim',  [ -obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).xdata( idx ), obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).xdata( idx ) ] , ...         
                                         'YLim',   [ -obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).ydata( idx ), obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).ydata( idx ) ] , ...    
                                         'ZLim',   [ -obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).zdata( idx ), obj.zoomSize + obj.markers{ iidx }( obj.zoomIdx ).zdata( idx ) ] )  
            end
           
        end
           
        
        function addTrackingPlots( obj, idx, plotLine )
            %addZoomWindow: Add a Zoom-in view plot of the main axes plot
            % [INPUTS]            
            %   (1) idx [integer]
            %       -  2 (Lower-right), 3 (Upper-right)            
            %   (2) tdata, ydata [float array]
            %       -  The data array for the tracker `
            obj.adjustFigures( idx )                     
            obj.addGraphicObject( idx, plotLine )

            % Add the tracking markers. 
            tmp = myMarker( plotLine.x, plotLine.y, zeros( 1, length( plotLine.x ) ), ...
                                                   'markersize', 4 * plotLine.lineWidth , ...
                                                  'markercolor', plotLine.lineColor );
            obj.addGraphicObject( idx, tmp );
        end
        
        function addZoomWindow( obj, idx, whichMarker, size )
            %addZoomWindow: Add a Zoom-in view plot of the main axes plot
            % [INPUTS]            
            %   (1) idx [integer]
            %       -  1 (Main), 2 (Lower-right), 3 (Upper-right)            
            %   (2) whichMarker [integer/string]
            %       -  The Marker that we want to focus
            %   (3) size 
            %       -  The size of the zoom-in window.
            
            obj.adjustFigures( idx )         

            if     isstring( whichMarker )     % If markers info. are given as strings
                
                idxM = find( strcmp( [ obj.markers{ 1 }.name ], whichMarker ) );   
                
            elseif isnumeric( whichMarker )    
                
                idxM = whichMarker; 
                
            end
            
            % Copy the graphic objects from main to the plot
            obj.isZoomed( idx ) = true;
            obj.zoomIdx  = idxM; 
            obj.zoomSize = size;
            
            % Copy and overwrite all the graphic objects from main to side plot
            % Need to use "copyobj" for the plot
            obj.hArrows{   idx } = copyobj( obj.hArrows{   1 }, obj.hAxes{ idx } );
            obj.hEllipses{ idx } = copyobj( obj.hEllipses{ 1 }, obj.hAxes{ idx } );
            obj.hMarkers{  idx } = copyobj( obj.hMarkers{  1 }, obj.hAxes{ idx } );
            obj.hLines{    idx } = copyobj( obj.hLines{    1 }, obj.hAxes{ idx } );    
            
            obj.arrows{   idx }  = obj.arrows{   1 };
            obj.ellipses{ idx }  = obj.ellipses{ 1 };
            obj.markers{  idx }  = obj.markers{  1 };
            obj.lines{    idx }  = obj.lines{    1 };             
            
            
        end
        
    end

end

