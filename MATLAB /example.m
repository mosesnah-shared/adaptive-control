% [Project]        [2.165] Robotics (Prof. JJE Slotine)
% [Title]          Applying adaptive control to the robot 
% [Author]         Moses C. Nah
% [Creation Date]  Sunday, October 18th, 2020
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 
addpath( './myGraphics' ); addpath( './myUtils' );
myFigureConfig( 'fontsize', 20, ...
               'lineWidth', 5, ...
              'markerSize', 25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();                        

%% ==================================================================
%% (1-) Calculating the Manipulator equation and the Y matrix / a vector
%% -- (1A) Set the 2DOF Robot

% To use the 2DOF robot, use the following line
robot = my2DOFRobot( );     

% To use the 4DOF robot, use the following line
% robot = my4DOFRobot( );     

robot.initialize( );

%% -- (1B) Calculate the M, C matrix and G vector + Calculate the Y matrix and a vector for the adaptive controller 

[ M, C, G ] = robot.deriveManipulatorEquation(  );
[ ~, Y, a ] = robot.findManipulatorRegressor(   );                        % You need to copy and paste this value to the mujoco simulation

%% -- (1C) PYTHON Conversion process
%       Skip this section if you are not interested in PYTHON implementation
%       In case you are using python, we need to copy-and-paste the Y matrix..
%       This section does the job for you.

tmpY = arrayfun( @char, Y, 'uniform', 0 );

% oldS: MATLAB Style print,   
oldS = [ string( [ robot.q, robot.dq, robot.dqr, robot.ddqr, robot.M, robot.L, robot.Lc ] ), "sin", "cos", "^2"  ];

% newS: python Style print
str1 = [ "q"; "dq"; "dqr"; "ddqr" ];                                       
str2 = "[" + string( ( 0 : robot.nDOF - 1 ) ) + "]";                       % Generating the string array, from 1 ~ nDOF                                                                       
str3 = [ "M"; "L"; "Lc" ];                                       
str4 = "[" + string( ( 0 : 1 ) ) + "]";                                    % Generating the string array, from 1 ~ nDOF                                                                       

newS = [ reshape(  append( str1, str2 )', 1, [] ), ...
         reshape(  append( str3, str4 )', 1, [] ), "np.sin", "np.cos", "**2" ];  % Making the mixture of strings for convertion

for i = 1 : length( oldS )
    tmpY = strrep( tmpY, oldS{ i }, newS{ i } );                           % Replacing 
end

[nr, nc] = size( Y );

for i = 1 : nr
    for j = 1 : nc 
        fprintf( 'self.Y[%d, %d] = %s\n', i-1, j-1, tmpY{ i, j }  )
    end
end

%% ==================================================================
%% (2-) Running the simulation in MATLAB
%          [WARNING] If you run for 4DOF robot case, it is extremely slow!
%% -- (2A) Calculate the M, C matrix and G vector, as a function of q

%                                                   Straighten the I matrix to a row vector
sym_array = [ robot.M, robot.L, robot.Lc, reshape( robot.I', 1, [] ), robot.g ];
val_array = { 1.0, 1.0, ... Mass   of each limb segment, ordered from proximal to distal (upperarm - forearm)
              1.0, 1.0, ... Length of each limb segment, ordered from proximal to distal (upperarm - forearm)
              0.5, 0.5, ... Length from proximal joint to center of mass, ordered from proximal to distal (upperarm - forearm)
               1, 1, 1, ... Moment of inertia of I1xx, I1yy, I1zz, w.r.t. center of mass.
               1, 1, 1, ... Moment of inertia of I2xx, I2yy, I2zz, w.r.t. center of mass.
                  9.81 };  % Gravity
              
M_val = subs( M, sym_array, val_array );
C_val = subs( C, sym_array, val_array );
G_val = subs( G, sym_array, val_array );

% For animation, the forward kinematics of the elbow and end-effector joint
pEL = robot.forwardKinematics( 2, [ 0; 0;             0 ] );               % Position of the elbow
pEE = robot.forwardKinematics( 2, [ 0; 0; -robot.L( 2 ) ] );               % Position of the end-effector

% Substituting the symbol's values to values
pEL = subs( pEL, sym_array, val_array );
pEE = subs( pEE, sym_array, val_array );                    

%% -- (2B) Do the ode45 calculation

T     = 10;                                                               
tspan = [0, T];

% Setting the initial condition of the robot
if     robot.nDOF == 2
    y0 = [ 0.6, 0.6, ... Initial joint position, relative angle coordinate.
           0.0, 0.0 ]; % Initial joint velocity
elseif robot.nDOF == 4
    y0 = [ 0.0, 0.1, 0.0, 0.1, ... Initial joint position, relative angle coordinate.
           0.0, 0.0, 0.0, 0.0 ]; % Initial joint velocity
end

[ t,y ] = ode45( @(t,y) odefcn( t, y, robot, M_val, C_val, G_val ), tspan, y0 );

t = t'; y = y';

%% -- (2C) Run the animation
% Since ode45 varies the step size, changing it to equivalent step size
dt    = .017;
tVec  = dt * ( 1 : T/dt ); 
qMat  = interp1( t, y( 1 : 2 * robot.nDOF, : )', tVec )';                  % Filling in the missing points for the animation

pSH_val  = zeros( 3, length( tVec ) ); 
pEL_val  = double( subs( pEL, robot.q, num2cell( qMat( 1 : robot.nDOF, : ), 2 )' ) );
pEE_val  = double( subs( pEE, robot.q, num2cell( qMat( 1 : robot.nDOF, : ), 2 )' ) );   

pos = { pSH_val, pEL_val, pEE_val };

% Marker in order, upper limb 3 joints( SH, EL, WR) 
stringList = [ "SH", "EL", "EE" ];                                         % 3 Upper limb markers 
sizeList   = [ 24, 24, 24 ];                                               % Setting the size of each markers
colorList  = [ c.pink; c.green; c.blue ];                                  % Setting the color of each markers

for i = 1 : length( pos )
   gObjs( i ) = myMarker( 'XData', pos{ i }( 1, : ), ... 
                          'YData', pos{ i }( 2, : ), ...
                          'ZData', pos{ i }( 3, : ), ...
                           'name', stringList( i ) , ...
                       'SizeData',  800            , ...
                      'LineWidth',   3             , ...
                'MarkerEdgeColor',  colorList( i, : ) );                   % Defining the markers for the plot

end

ani = myAnimation( dt, gObjs );                                            % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array

ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'Color', c.grey )        
tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [ 0, 0 ]     )           

ani.addZoomWindow( 3 , "EE", 1);   
set( ani.hAxes{ 3 }, 'view',   [ 0, 0 ]     )   

ani.run( 1, T, false, 'output')

