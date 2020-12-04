% [Project]        [2.165] Robotics (Prof. JJE Slotine)
% [Title]          Applying adaptive control to the robot 
% [Author]         Moses C. Nah
% [Creation Date]  Sunday, October 18th, 2020
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 
addpath( './myGraphics' );
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

%% (1B) Calculate the M, C matrix and G vector + Calculate the Y matrix and a vector for the adaptive controller 

[M, C, G]   = robot.deriveManipulatorEquation(  );
[ ~, Y, a ] = robot.findManipulatorRegressor(   );                        % You need to copy and paste this value to the mujoco simulation

%% (1C) PYTHON Conversion process
%       In case you are using python, we need to copy-and-paste the Y matrix..
%       This section does the job for you



[nr, nc] = size( Y );

for i = 1 : nr
    for j = 1 : nc 
        fprintf( 'self.Y[%d, %d] = %s\n', i-1, j-1, tmp{ i, j }  )
    end
end

%% ==================================================================
%% (2-) Running the simulation in MATLAB
%          [WARNING] If you run for 4DOF robot case, it is extremely slow!
%% -- (2B) Do the ode45 calculation
T     = 10;                                                               
tspan = [0, T];

if     robot.nDOF == 2
    
elseif robot.nDOF == 4
    
end

[tvec,y] = ode45( @(t,y) odefcn(t, y, robot.M_val, robot.C_val, robot.G_val ), tspan, y0 );


tvec = tvec'; y = y';
%% -- (1C) Run the Animation
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

