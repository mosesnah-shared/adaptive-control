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
robot = my2DOFRobot(  );  %'length', [0.294, 0.291]

% robot.jacobian( 2,  , 'sym')
pos = robot.forwardKinematics( 2, [ robot.L(2);0;0] );
% robot.jacobian( 1, [ robot.L(1);0;0] )
M = robot.getM( );      % Mass Matrix 
C = robot.getC( );      % Coriolis Term
G = robot.getG( );      % Gravity Matrix

M_val = robot.substitute( M, 'length', [1,1], 'mass', [1,1], 'lengthCOM', [0.5, 0.5], 'inertia', [0, 0, 0.2; 0, 0, 0.2], 'gravity', 9.81  );
C_val = robot.substitute( C, 'length', [1,1], 'mass', [1,1], 'lengthCOM', [0.5, 0.5], 'inertia', [0, 0, 0.2; 0, 0, 0.2], 'gravity', 9.81  );
G_val = robot.substitute( G, 'length', [1,1], 'mass', [1,1], 'lengthCOM', [0.5, 0.5], 'inertia', [0, 0, 0.2; 0, 0, 0.2], 'gravity', 9.81  );

%% -- (1 - b) Do the calculation\
T = 10;                                                                    % Total Time of ode intergration

tspan = [0, T];
y0 = [1, 1, 0, 0];
[tvec,y] = ode45( @(t,y) odefcn(t, y, M_val, C_val, G_val ), tspan, y0 );


tvec = tvec'; y = y';
%% -- (1 - c) Run the Animation
% Since ode45 varies the step size, changing it to equivalent step size
tstep = 1e-2;
tmp = tstep * ( 1 : T/tstep ); 
tVec = tmp;

qMat = interp1( tvec, y( 1:4, : )', tmp ); 
% tmp = arrayfun( xd, tVec , 'UniformOutput', false); tmp = cell2mat( tmp' );

q1 = qMat( :, 1 )'; q2 = qMat( :, 2 )';

pSH = zeros( 3, length( tVec ) );
pEL = zeros( 3, length( tVec ) );
pEE = zeros( 3, length( tVec ) );

eqEL = robot.forwardKinematics( 1, [ 1; 0; 0 ] );
eqEE = robot.forwardKinematics( 2, [ 1; 0; 0 ] ); 
eqEE = subs( eqEE, 'L1', 1 );

pEL = double( reshape( subs( eqEL, {'q1', 'q2'}, { qMat( :, 1 ), qMat( :, 2 ) } ), length( q1 ), 3 )' );
pEE = double( reshape( subs( eqEE, {'q1', 'q2'}, { qMat( :, 1 ), qMat( :, 2 ) } ), length( q1 ), 3 )' );


%%

stringList = [ "SH", "EL", "EE" ];                                         % 3 Upper limb markers 

% Marker in order, target (1), upper limb (3, SH, EL, WR) and Whip (25) 
sizeList   = [ 24, 24, 24 ];                        % Setting the size of each markers
colorList  = [ repmat( c.pink, 3, 1) ];  % Setting the color of each markers


markers(1 ) = myMarker( pEE( 1, : ), pEE( 2, :) , pEE(3, : ), ...
                                          'name', stringList( 3 ) , ...
                                    'markersize',   sizeList( 3 ) , ...
                                   'markercolor',  colorList( 3, : ) );    % Defining the markers for the plot

markers(2 ) = myMarker( pEL( 1, : ), pEL( 2, :) , pEL(3, : ), ...
                                          'name', stringList( 2 ) , ...
                                    'markersize',   sizeList( 2 ) , ...
                                   'markercolor',  colorList( 2, : ) );    % Defining the markers for the plot
                               
markers(3 ) = myMarker( pSH( 1, : ), pSH( 2, :) , pSH(3, : ), ...
                                          'name', stringList( 1 ) , ...
                                    'markersize',   sizeList( 1 ) , ...
                                   'markercolor',  colorList( 1, : ) );    % Defining the markers for the plot

                               
ani = my3DAnimation( 0.01, markers );                                        % Input (1) Time step of sim. (2) Marker Information
ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'linecolor', c.grey )        


tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0   90 ]     )                  % Set the view, xlim, ylim and zlim of the animation
                                                                           % ani.hAxes{ 1 } is the axes handle for the main animation
                 
                       
ani.run( 0.2, false, 'output')
                                                                           

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