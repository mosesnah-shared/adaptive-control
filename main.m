% [Project]        [2.165] Robotics (Prof. JJE Slotine)
% [Title]          Applying adaptive control to the robot 
% [Author]         Moses C. Nah
% [Creation Date]  Sunday, October 18th, 2020
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 

myFigureConfig( 'fontsize', 20, ...
               'lineWidth',  5, ...
              'markerSize', 25    )     
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();                        

%% (--) ========================================================
%% (1-) 2DOF Robot Simulation
%% --- (1 - a) Parsing the txt File 
robot = my2DOFRobot( 'L',      [1,1], ...
                     'M',      [1,1], ...
                    'Lc', [0.5, 0.5], ...
                     'I', [1, 1, 1; 1, 1, 1], ...
                     'g', 9.81  );

%% -- (1 - b) Do the calculation
T = 10;                                                                    % Total Time of ode intergration

tspan = [0, T];
y0 = [0.1, 0.1, 0, 0];
[tvec,y] = ode45( @(t,y) odefcn(t, y, robot.M_val, robot.C_val, robot.G_val ), tspan, y0 );


tvec = tvec'; y = y';
%% -- (1 - c) Run the Animation
% Since ode45 varies the step size, changing it to equivalent step size
tstep = 1e-2;
tmp = tstep * ( 1 : T/tstep ); 
tVec = tmp;

qMat = interp1( tvec, y( 1:4, : )', tmp ); 
% tmp = arrayfun( xd, tVec , 'UniformOutput', false); tmp = cell2mat( tmp' );

pSH = zeros( 3, length( tVec ) );
pEL = robot.calcForwardKinematics( 1, [1;0;0], qMat(:,1:2)' );
pEE = robot.calcForwardKinematics( 2, [1;0;0], qMat(:,1:2)' );

pos = {pSH, pEL, pEE};


stringList = [ "SH", "EL", "EE" ];                                         % 3 Upper limb markers 

% Marker in order, target (1), upper limb (3, SH, EL, WR) and Whip (25) 
sizeList   = [ 24, 24, 24 ];                        % Setting the size of each markers
colorList  = [ c.pink; c.green; c.blue ];  % Setting the color of each markers

for i = 1 : length( pos )
   markers( i ) = myMarker( pos{ i }( 1, : ), pos{ i }( 2, :) , pos{ i }(3, : ), ...
                                                       'name', stringList( i ) , ...
                                                 'markersize',   sizeList( i ) , ...
                                                'markercolor',  colorList( i, : ) );    % Defining the markers for the plot

end


ani = my3DAnimation( 0.01, markers );                                        % Input (1) Time step of sim. (2) Marker Information
ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'linecolor', c.grey )        


tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0, 0 ]     )           
                                                      
                              
ani.run( 1, true, 'output')
                                                                           

%% Deriving the end-effector time derivative of Jacobian

Jsym  = robot.jacobian( 2, [robot.L(2);0;0] );
dJsym = diff( Jsym,  't' );

tmp = subs(   robot.G_mat,  diff( robot.q, robot.t ), robot.dq )
dJEEMat = arrayfun(@char, tmp, 'uniform', 0);

oldS = {'q1(t)','q2(t)', 'dq1(t)','dq2(t)', 'sin'   , 'cos'   , 'L1'     , 'L2'      , 'Lc1'      , 'Lc2'     , 'M1'      , 'M2'      };
newS = { 'q[0]', 'q[1]',  'dq[0]', 'dq[1]', 'np.sin', 'np.cos', 'self.L1', 'self.L2' , 'self.Lc1' , 'self.Lc2', 'self.m1' , 'self.m2' };


tmp = dJEEMat;
for i = 1 : length(oldS)
    tmp = strrep( tmp, oldS{i}, newS{i} );
end
     
%% (--) ========================================================
%% (2-) 2DOF Robot Simulation
%% --- (2 - a) Parsing the txt File 

data = myTxtParse( 'data_log.txt' );

% dEE = robot.calcForwardKinematics( 2, [1;0;0], data.desiredJointPos );


stringList = [ "SH", "EL", "EE" ];                                         % 3 Upper limb markers 

% Marker in order, target (1), upper limb (3, SH, EL, WR) and Whip (25) 
sizeList   = [ 24, 24, 24 ];                        % Setting the size of each markers
colorList  = [ c.blue; c.green; c.pink ];  % Setting the color of each markers

pSH = data.geomXYZPositions(1:3, :);
pEL = data.geomXYZPositions(4:6, :);
pEE = data.geomXYZPositions(7:9, :);
% pADD = data.geomXYZPositions(10:12, :);


markers( 1 ) = myMarker( pEE( 1, : ), pEE( 2, :) , pEE(3, : ), ...
                                          'name', stringList( 3 ) , ...
                                    'markersize',   sizeList( 3 ) , ...
                                   'markercolor',  colorList( 3, : ) );    % Defining the markers for the plot

markers( 2 ) = myMarker( pEL( 1, : ), pEL( 2, :) , pEL(3, : ), ...
                                          'name', stringList( 2 ) , ...
                                    'markersize',   sizeList( 2 ) , ...
                                   'markercolor',  colorList( 2, : ) );    % Defining the markers for the plot
                               
markers(3  ) = myMarker( pSH( 1, : ), pSH( 2, :) , pSH(3, : ), ...
                                          'name', stringList( 1 ) , ...
                                    'markersize',   sizeList( 1 ) , ...
                                   'markercolor',  colorList( 1, : ) );    % Defining the markers for the plot
% 
% markers( 4  ) = myMarker( dEE( 1, : ), dEE( 2,  :) ,  dEE( 3, :)  ,...
%                                           'name', stringList( 1 ) , ...
%                                     'markersize',   sizeList( 1 ) * 0.4 , ...
%                                    'markercolor',  c.yellow );    % Defining the markers for the plot
%                                
markers( 4  ) = myMarker( data.desiredEEPos( 1, : ), zeros(1, length( data.desiredEEPos ) ),  data.desiredEEPos( 2, :) ,...
                                          'name', stringList( 1 ) , ...
                                    'markersize',   sizeList( 1 ) * 0.4 , ...
                                   'markercolor',  c.yellow );    % Defining the markers for the plot

% markers( 5  ) = myMarker( pADD( 1, : ), pADD( 2, :),  pADD( 3, :) ,...
%                                           'name', stringList( 1 ) , ...
%                                     'markersize',   sizeList( 1 ) * 0.4 , ...
%                                    'markercolor',  c.blue_sky );    % Defining the markers for the plot                               
                               
                               
tmp = sqrt( ( pEE(1,:) - data.desiredEEPos(1,:) ).^2 + ( pEE(3,:) - data.desiredEEPos(2,:) ).^2 );
tmp1 = my2DLine( data.currentTime, tmp, 'linecolor', c.pink,   'linestyle', '-', 'linewidth', 6 );

% tmp = sqrt( ( pEE(1,:) - dEE(1,:) ).^2 + ( pEE(3,:) - dEE(3,:) ).^2 );
% tmp1 = my2DLine( data.currentTime, tmp, 'linecolor', c.pink,   'linestyle', '-', 'linewidth', 6 );


ani = my3DAnimation( data.currentTime(2), markers );                                        % Input (1) Time step of sim. (2) Marker Information
ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'linecolor', c.grey )        

ani.addTrackingPlots( 2, tmp1 );                                
                       
tmp = 0:0.01:2*pi;
plot3( cos( tmp ) , zeros( 1, length( tmp ) ) , sin( tmp ) , 'linestyle', '--', 'linewidth', 1, 'parent', ani.hAxes{ 1 } ) ;

tmpLim = 4.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0   0 ]     )                  % Set the view, xlim, ylim and zlim of the animation
                                                                           % ani.hAxes{ 1 } is the axes handle for the main animation     
ani.run( .5, true, 'output')
                                                                  
                              


%% (-3) ODE Function Definition

function dx = odefcn( t, x, M, C, G )
    % v = xr, qr, q, qe, each a 2-by-1 vector
    % Since we know q, we can calculate x, which is the actual end-effector position
    q1  = x( 1 );  q2  = x( 2 );
    dq1 = x( 3 );  dq2 = x( 4 );
    
    tmpM = double( subs( M, {'q1', 'q2' }, { q1, q2 }  ) );
    tmpC = double( subs( C, {'q1', 'q2', 'dq1', 'dq2' }, { q1, q2, dq1, dq2 } ) );
    tmpG = double( subs( G, {'q1', 'q2' }, { q1, q2 } ) );
    
    ddq = tmpM \ ( -tmpC * x(3:4) - tmpG' );
    dx = [dq1; dq2; ddq];
end