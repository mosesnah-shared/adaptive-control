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

robot = my2DOFRobot( 'length', [0.294, 0.291] );


% robot.jacobian( 2,  , 'sym')
