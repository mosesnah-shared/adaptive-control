%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 

myFigureConfig( 'fontsize', 40, ...
               'lineWidth', 7, ...
              'markerSize', 25    )
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();       

%% IMAGE 1
data = myTxtParse( 'data_log1.txt' );
figure()
hold on
p1 = plot( data.currentTime, sqrt( ( data.desiredEEPos(1,:) -  data.jointAngleActual(1,:) ).^2 ), '-', 'color', c.orange );
p2 = plot( data.currentTime, sqrt( ( data.desiredEEPos(2,:) -  data.jointAngleActual(2,:) ).^2 ), '-', 'color', c.blue   );
set( gca, 'xlim', [0, 15], 'ylim', [0,1.6])
xlabel( 'Time [sec]' ); ylabel( 'Position Error [rad]' )

legend( 'Shoulder', 'Elbow' )
mySaveFig( gcf, '2DOF_Pos_Error' )

%% IMAGE 2
robot = my2DOFRobot( 'L',      [1,1], ...
                     'M',      [1,1], ...
                    'Lc', [0.5, 0.5], ...
                     'I', [1, 1, 1; 1, 1, 1], ...
                     'g', 9.81    );
                 
[tau, Y, a] = robot.findManipulatorRegressor( );             



tmpa = data.a./a_real';
figure( )
for i = 1:length( a ) 
    
    p = plot( data.currentTime, tmpa( i, : )); 
    p.Color(4) = 0.7;
    set( gca, 'xlim', [30, 60], 'ylim', [0, 2] )
    hold on
end

yline( 1, 'linewidth', 5, 'linestyle', '--' )

xlabel ( 'Time [sec]' ); ylabel( 'Parameter Estimation Error [-]' );
legend ( '$a_1$', '$a_2$', '$a_3$', '$a_4$', '$a_5$' )  
mySaveFig( gcf, ['output', num2str( i )] )


%% IMAGE 3
data = myTxtParse( 'data_log.txt' );
figure()
hold on
p1 = plot( data.currentTime, sqrt( ( data.desiredEEPos(1,:) -  data.jointAngleActual(1,:) ).^2 ), '-', 'color', c.orange );
p2 = plot( data.currentTime, sqrt( ( data.desiredEEPos(2,:) -  data.jointAngleActual(2,:) ).^2 ), '-', 'color', c.blue   );
p3 = plot( data.currentTime, sqrt( ( data.desiredEEPos(3,:) -  data.jointAngleActual(3,:) ).^2 ), '-', 'color', c.green  );
p4 = plot( data.currentTime, sqrt( ( data.desiredEEPos(4,:) -  data.jointAngleActual(4,:) ).^2 ), '-', 'color', c.pink   );
set( gca, 'xlim', [0, 30], 'ylim', [0,1.6])
xlabel( 'Time [sec]' ); ylabel( 'Position Error [rad]' )

alpha = 0.7;
p1.Color(4) = alpha;
p2.Color(4) = alpha;
p3.Color(4) = alpha;
p4.Color(4) = alpha;


legend( 'J1', 'J2', 'J3', 'J4')
mySaveFig( gcf, '4DOF_Pos_Error' )

%% IMAGE 4
robot = my4DOFRobot( 'L',      [1,1], ...
                     'M',      [1,1], ...
                    'Lc', [0.5, 0.5], ...
                     'I', [1, 1, 1; 1, 1, 1], ...
                     'g', 9.81    );
                 
[tau, Y, a] = robot.findManipulatorRegressor( );             

a_real = double( robot.substitute( a, { 'M', 'L', 'Lc', 'I', 'g' }, robot.r ) );
% data = myTxtParse( 'data_log.txt' );

tmpa = data.a./a_real';
    
tmpc = cell2mat( struct2cell( c ) );
tmpc( 7, : ) = [];

figure( )
hold on
for i = 1:length( a ) 
    p = plot( data.currentTime, tmpa(i, : ), 'color', tmpc(i, :) ); 
    p.Color( 4 ) = 0.7;
%     mySaveFig( gcf, ['output', num2str( i )] )
end

yline( 1, 'linewidth', 5, 'linestyle', '--' )

set( gca, 'xlim', [30, 120] )
xlabel( 'Time [sec]' ); ylabel( 'Parameter Estimation Error [-]' )
legend( '$a_1$', '$a_2$', '$a_3$', '$a_4$', '$a_5$', ...
        '$a_6$', '$a_7$', '$a_8$', '$a_9$', '$a_{10}$', '$a_{11}$', 'location', 'eastoutside')
mySaveFig( gcf, '4DOF_Par_Est_Error' )

%% IMAGE 5 Cartesian

data = myTxtParse( 'data_log.txt' );

figure( )
hold on

pEE = data.geomXYZPositions( 7:9, : );
pd  = data.desiredEEPos;

perror = pEE - pd;


for i = 1:3
    p = plot( data.currentTime, perror(i, : ) ); 
    p.Color( 4 ) = 0.7;
%     mySaveFig( gcf, ['output', num2str( i )] )
end

yline( 0, 'linewidth', 5, 'linestyle', '--' )

% set( gca, 'xlim', [30, 120] )
xlabel( 'Time [sec]' ); ylabel( 'Cartesian Position Error [m]' )
legend( 'x', 'y', 'z', 'location', 'southeast')
mySaveFig( gcf, '4DOF_Cartesian_Error' )


%% 
data = myTxtParse( 'data_log.txt' );
figure( )
hold on

pEE = data.geomXYZPositions( 7:9, : );
pd  = data.desiredEEPos;

perror = pEE - pd;

p1 = plot( pEE( 1, : ), pEE( 2,: ), 'linewidth', 8, 'color', c.blue_sky );
p1.Color( 4 ) = 0.7;
p2 = plot(  pd( 1, : ),  pd( 2,: ), '--', 'linewidth', 4, 'color', c.pink );
p1.Color( 4 ) = 0.7;

% set( gca, 'xlim', [30, 120] )
xlabel( 'X [m]' ); ylabel( 'Y [m]' )
legend( 'Actual End-effector', 'Desired Trajectory', 'location', 'southeast')
mySaveFig( gcf, '4DOF_Cartesian_Trace' )

