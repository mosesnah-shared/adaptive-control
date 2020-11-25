
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
robot = my4DOFRobot( 'L',      [1,1], ...
                     'M',      [1,1], ...
                    'Lc', [0.5, 0.5], ...
                     'I', [1, 1, 1; 1, 1, 1], ...
                     'g', 9.81    );
                 
[tau, Y, a] = robot.findManipulatorRegressor( );             


a_real = double( robot.substitute( a, { 'M', 'L', 'Lc', 'I', 'g' }, robot.r ) );
data = myTxtParse( 'data_log1.txt' );

    
figure( )
for i = 1:length( a ) 
    plot( data.currentTime, data.a( i, : ) - a_real( i ) ); 
    hold on
%     yline( a_real( i ), 'linewidth', 5, 'linestyle', '--' )
%     mySaveFig( gcf, ['output', num2str( i )] )
end

xlabel( 'Time [sec]' ); ylabel( 'Parameter Estimation Error [-]' )
legend( '$a_1$', '$a_2$', '$a_3$', '$a_4$', '$a_5$')
mySaveFig( gcf, '2DOF_Par_Est_Error' )