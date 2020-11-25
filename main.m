% [Project]        [2.165] Robotics (Prof. JJE Slotine)
% [Title]          Applying adaptive control to the robot 
% [Author]         Moses C. Nah
% [Creation Date]  Sunday, October 18th, 2020
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 

% myFigureConfig( 'fontsize', 40, ...
%                'lineWidth', 10, ...
%               'markerSize', 25    )
          
myFigureConfig( 'fontsize', 20, ...
   'lineWidth', 5, ...
  'markerSize', 25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();                        

%% (--) ========================================================
%% (1-) 2DOF Robot Simulation
%% --- (1 - a) Parsing the txt File 
robot = my4DOFRobot( 'L',      [1,1], ...
                     'M',      [1,1], ...
                    'Lc', [0.5, 0.5], ...
                     'I', [1, 1, 1; 1, 1, 1], ...
                     'g', 9.81  );

robot = my2DOFRobotAndRod( 'L',      [1,1], ...
                           'M',      [1,1], ...
                          'Lc', [0.5, 0.5], ...
                           'I', [1, 1, 1; 1, 1, 1], ...
                           'g', 9.81    );
                 %%
% robot = my4DOFRobot( );                 
[tau, Y, a] = robot.findManipulatorRegressor( );             

%% (--) ========================================================
%% (2-) 2DOF Robot Simulation
%% --- (2 - a) Parsing the txt File 

data = myTxtParse( 'data_log.txt' );

% dEE = robot.calcForwardKinematics( 2, [1;0;0], data.desiredJointPos );


stringList = [ "SH", "EL", "EE" ];                                         % 3 Upper limb markers 


sizeList   = 50 * ones( 1, 3 ) ;                       


pSH = data.geomXYZPositions(1:3, :);
pEL = data.geomXYZPositions(4:6, :);
pEE = data.geomXYZPositions(7:9, :);

% Syntax D for Desired
% pEL_D   = robot.calcForwardKinematics( 2, [0;0;0], data.desiredEEPos );
% pEE_D   = robot.calcForwardKinematics( 2, [0;0;-0.291], data.desiredEEPos );
% pSH_D   = zeros( length( robot.q ), length( data.desiredEEPos ) );


markers( 1 ) = myMarker( pEE( 1, : ), pEE( 2, :) , pEE(3, : ), ...
                                     'name', stringList( 3 ) , ...
                               'markersize',   sizeList( 3 ) , ...
                              'markercolor',  c.blue );    

markers( 2 ) = myMarker( pEL( 1, : ), pEL( 2, :) , pEL(3, : ), ...
                                     'name', stringList( 2 ) , ...
                               'markersize',   sizeList( 2 ) , ...
                                   'markercolor',  c.blue );   
                               
markers( 3 ) = myMarker( pSH( 1, : ), pSH( 2, :) , pSH(3, : ), ...
                                     'name', stringList( 1 ) , ...
                               'markersize',   sizeList( 1 ) , ...
                                   'markercolor',  c.blue  );  

markers( 4 ) = myMarker( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :) , data.desiredEEPos(3, : ), ...
                                     'name',  "EE_D" , ...
                               'markersize',   20 , ...
                              'markerAlpha',           0.5, ...                               
                              'markercolor',  c.pink );    
                               
% markers( 4 ) = myMarker( pEE_D( 1, : ), pEE_D( 2, :) , pEE_D(3, : ), ...
%                                      'name',  "EE_D" , ...
%                                'markersize',   sizeList( 3 ) , ...
%                               'markerAlpha',             0.5, ...                               
%                               'markercolor',  c.pink );    
% 
% markers( 5 ) = myMarker( pEL_D( 1, : ), pEL_D( 2, :) , pEL_D(3, : ), ...
%                                      'name', "EL_D" , ...
%                                'markersize',   sizeList( 2 ) , ...
%                               'markerAlpha',             0.5, ...                               
%                               'markercolor',  c.pink );   
%                                
% markers( 6 ) = myMarker( pSH_D( 1, : ), pSH_D( 2, :) , pSH_D(3, : ), ...
%                                      'name', "SH_D" , ...
%                                'markersize',   sizeList( 1 ) , ...
%                               'markerAlpha',             0.5, ...
%                               'markercolor',  c.pink  );  
                               
              
ani = my3DAnimation( data.currentTime(2), markers );                       % Input (1) Time step of sim. (2) Marker Information
ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'linecolor', c.grey )        
% ani.connectMarkers( 1, [ "SH_D", "EL_D", "EE_D" ], 'linecolor', c.grey, 'linestyle', '--' );        

% a_real = double( robot.substitute( a, { 'M', 'L', 'Lc', 'I', 'g' }, robot.r ) );

tmp1 = my2DLine( data.currentTime, sqrt( sum( ( data.geomXYZPositions( 7:9, : ) - data.desiredEEPos ).^2) ), 'linecolor', c.pink,   'linestyle', '-', 'linewidth', 6 );
plot3( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :),  data.desiredEEPos( 3, :) , 'color', c.blue, 'linestyle', '--', 'linewidth', 4, 'parent', ani.hAxes{ 1 } ) ;



ani.addTrackingPlots( 2, tmp1 );
% ani.addTrackingPlots( 2, tmp2 );
% ani.addTrackingPlots( 2, tmp3 );
% ani.addTrackingPlots( 2, tmp4 );

tmpLim = 0.6;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [32.8506   15.1025 ] )

set( ani.hAxes{ 2 }, 'XLim', [0, 6] )
                 
ani.addZoomWindow( 3 , "EE", 0.2);       
set( ani.hAxes{ 3 }, 'xticklabel', [], 'yticklabel', [], 'zticklabel', [])
set( ani.hAxes{ 3 }, 'view',   [32.8506   15.1025 ] )

plot3( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :),  data.desiredEEPos( 3, :) , 'color', c.blue, 'linestyle', '--', 'linewidth', 4, 'parent', ani.hAxes{ 3 } ) ;
ani.run( 0.5, 6, true, 'output')
                                                                  
                              

%% (--) ========================================================
%% (3-) Manipulator Regressor Identification
%% --- (3 - a) Finding the Y function for the calculation.

robot4DOF = my4DOFRobot( );                
                 
[tau, Y, a] = robot4DOF.findManipulatorRegressor( );

%%
data = myTxtParse( 'data_log.txt' );
a_real = double( robot.substitute( a, { 'M', 'L', 'Lc', 'I', 'g' }, robot.r ) );
tmpc = cell2mat( struct2cell( c ) );


for i = 1:length( a ) 
    figure( )
    plot( data.currentTime, data.a( i, : ) ); 
    hold on
    yline( a_real( i ), 'linewidth', 5, 'linestyle', '--' )
%     mySaveFig( gcf, ['output', num2str( i )] )
end

%% Printing in a format where it makes it easy to put the Y matrix 

[nr, nc] = size( Y );

for i = 1 : nr
    for j = 1 : nc 
        fprintf( 'self.Y[%d, %d] = %s\n', i-1, j-1, tmp{ i, j }  )
    end
end
%% Check sufficient richness

syms t
q1 = 0.3 * sin( t ) + 0.3 * sin( 2*t ) + 0.3 * sin( 3*t ) + 0.3 * sin( 4*t ) + 0.3 * sin( 5*t ) + 0.3 * sin( 6*t );
q2 = pi/2 + 0.3 * sin( t ) + 0.3 * sin( 2*t ) + 0.3 * sin( 3*t ) + 0.3 * sin( 4*t ) + 0.3 * sin( 5*t ) + 0.3 * sin( 6*t );
dq1 = diff( q1, 't');
dq2 = diff( q2, 't');
ddq1 = diff( dq1, 't');
ddq2 = diff( dq2, 't');


q1   = matlabFunction( q1 );
q2   = matlabFunction( q2 );
dq1  = matlabFunction( dq1 );
dq2  = matlabFunction( dq2 );
ddq1 = matlabFunction( ddq1 );
ddq2 = matlabFunction( ddq2 );

dt = 1
tVec=0:dt:400;

q1_val   = q1( tVec );
q2_val   = q2( tVec );
dq1_val  = dq1( tVec );
dq2_val  = dq2( tVec );
ddq1_val = ddq1( tVec );
ddq2_val = ddq2( tVec );

Y_avg = zeros( length( a ), length( a ) );


for i = 1 : length( tVec )
    
    Y_tmp = double( subs( Y, {'ddqr1', 'ddqr2', 'q1', 'q2', 'dqr1', 'dqr2', 'dq1', 'dq2'}, ...
                     {ddq1_val( i ), ddq2_val( i ), q1_val( i ), q2_val( i ), dq1_val( i ), dq2_val( i ), dq1_val( i ), dq2_val( i ) }  ) ); 
                     
    Y_avg = Y_avg + dt * Y_tmp' * Y_tmp;  
end

 

%% (--) ========================================================
%% (4-) Substitution for Python code

dJEEMat = arrayfun(@char, Y, 'uniform', 0);

oldS = {'q1(t)','q2(t)', 'q3(t)','q4(t)', 'q5(t)', 'q6(t)', 'dq1(t)','dq2(t)', 'dq3(t)','dq4(t)', 'dq5(t)','dq6(t)', ...
        'ddqr1', 'ddqr2' , 'ddqr3', 'ddqr4', 'ddqr5', 'ddqr6' , 'dqr1', 'dqr2', 'dqr3', 'dqr4' ,  'dqr5','dqr6', ......
        'sin'   , 'cos'   , 'L1'     , 'L2'  , 'L3'    , 'Lc1'      , 'Lc2'   , 'Lc3',  'M1'      , 'M2'    , 'M3'  };
newS = { 'q[0]', 'q[1]', 'q[2]',  'q[3]', 'q[4]', 'q[5]', 'dq[0]', 'dq[1]', 'dq[2]', 'dq[3]','dq[4]', 'dq[5]',...
         'ddqr[0]', 'ddqr[1]', 'ddqr[2]', 'ddqr[3]',  'ddqr[4]', 'ddqr[5]', 'dqr[0]', 'dqr[1]',  'dqr[2]', 'dqr[3]',  'dqr[4]', 'dqr[5]',...
         'np.sin', 'np.cos', 'self.L1', 'self.L2' , 'self.L3' , 'self.Lc1' , 'self.Lc2', 'self.Lc3', 'self.m1' , 'self.m2', 'self.m3' };


tmp = dJEEMat;
for i = 1 : length(oldS)
    tmp = strrep( tmp, oldS{i}, newS{i} );
end


%% (--) 
%% Animation with a Whip

data = myTxtParse( 'data_log.txt' );

c_m = c.peach;
nodeN = 25;

genNodes = @(x) ( "node" + (1:x) );
stringList = [ "Target", "SH", "EL", "EE",  genNodes( 25 ) ];       % 3 Upper limb markers + 1 target

% Marker in order, target (1), upper limb (3, SH, EL, WR) and Whip (25) 
sizeList   = [ 24, 40, 40, 40, 12 * ones( 1, 25 ) ];                        % Setting the size of each markers
colorList  = [ c_m; repmat( c_m, 3, 1); repmat( c.grey, 25 , 1 ) ];  % Setting the color of each markers

for i = 1: nodeN    % Iterating along each markers
    markers( i ) = myMarker( data.geomXYZPositions( 3 * i - 2, : ), ... 
                             data.geomXYZPositions( 3 * i - 1, : ), ... 
                             data.geomXYZPositions( 3 * i    , : ), ...
                                          'name', stringList( i ) , ...
                                    'markersize',   sizeList( i ) , ...
                                   'markercolor',  colorList( i, : ) );    % Defining the markers for the plot
end

pEE =  data.geomXYZPositions( 10:12, :  );

markers( end + 1 ) = myMarker( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :),  data.desiredEEPos( 3, :) ,...
                                          'name', stringList( 1 ) , ...
                                    'markersize',   sizeList( 1 ) * 0.4 , ...
                                   'markercolor',  c.yellow );    % Defining the markers for the plot


ani = my3DAnimation( data.currentTime(2), markers );                                        % Input (1) Time step of sim. (2) Marker Information
ani.connectMarkers( 1, [ "SH", "EL", "EE" ], 'linecolor', c.grey )        
                                                                           % Connecting the markers with a line.

                                                                           
tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [32.8506   15.1025 ] )
%                      'view',   [41.8506   15.1025 ]     )                  % Set the view, xlim, ylim and zlim of the animation
                                 
tmp = sqrt( ( pEE(1,:) - data.desiredEEPos(1,:) ).^2 + ( pEE(2,:) - data.desiredEEPos(2,:) ).^2  + ( pEE(3,:) - data.desiredEEPos(3,:) ).^2 );
tmp1 = my2DLine( data.currentTime, tmp, 'linecolor', c.pink,   'linestyle', '-', 'linewidth', 6 );

ani.addTrackingPlots( 2, tmp1 );

plot3( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :),  data.desiredEEPos( 3, :) , 'color', c.yellow, 'linestyle', '--', 'linewidth', 1, 'parent', ani.hAxes{ 1 } ) ;
ani.addZoomWindow( 3 , "EE", 0.6);            

plot3( data.desiredEEPos( 1, : ), data.desiredEEPos( 2, :),  data.desiredEEPos( 3, :) , 'color', c.yellow, 'linestyle', '--', 'linewidth', 1, 'parent', ani.hAxes{ 3 } ) ;

set( ani.hAxes{ 3 }, 'view',  get( ani.hAxes{ 1 }, 'view' ) )
set( ani.hAxes{ 3 }, 'xticklabel', [], 'yticklabel', [], 'zticklabel', [])

ani.run( 0.5, 14, true, 'output') 

%% (--) ODE Function Definition

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