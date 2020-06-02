%% Generate Input Files For Isometric Extension Example
%==========================================================================
% Author: Colin Smith
%
%
%==========================================================================
import org.opensim.modeling.*

%% Simulation Time
%Simulation consists of four phases:
%settle1: allow knee to settle into equilbrium
%flex: hip and knee flexion
%settle2: allow knee to settle into equilbrium 
%force: ramp up the muscle forces

time_step = 0.01;

settle1_duration = 0.5;
flex_duration = 1.0;
settle2_duration = 0.5;
force_duration = 1.0;

settle1_time = 0 : time_step : settle1_duration;
flex_time = settle1_duration + time_step : time_step : settle1_duration + flex_duration;
settle2_time = settle1_duration + flex_duration + time_step : time_step : settle1_duration + flex_duration + settle2_duration;
force_time = settle1_duration + flex_duration + settle2_duration + time_step : time_step : settle1_duration + flex_duration + settle2_duration + force_duration;
time = [settle1_time, flex_time, settle2_time, force_time];

time_points = [0,settle1_duration,...
    settle1_duration + flex_duration,...
    settle1_duration + flex_duration + settle2_duration,...
    settle1_duration + flex_duration + settle2_duration + force_duration];

num_settle1_steps = length(settle1_time);
num_flex_steps = length(flex_time);
num_settle2_steps = length(settle2_time);
num_force_steps = length(force_time);
num_steps = length(time);


%% Prescribed Coordinates File
prescribed_coordinate_file = 'prescribed_coordinates.sto';

max_hip_flex = 30;
max_knee_flex = 30;

hip_flex = [0,0,max_hip_flex,max_hip_flex,max_hip_flex];
knee_flex = [0,0,max_knee_flex,max_knee_flex,max_knee_flex];

smooth_hip_flex = interp1(time_points, hip_flex, time,'pchip');
smooth_knee_flex = interp1(time_points, knee_flex, time,'pchip');

coord_data.hip_flex_r = smooth_hip_flex';
coord_data.knee_flex_r = smooth_knee_flex';
coord_data.time = time;

coord_table = osimTableFromStruct(coord_data); %% Function distributed in OpenSim 4.0 resources
STOFileAdapter.write(coord_table,prescribed_coordinate_file);



% Prescribed coordinates plot
coord_fig = figure('name','prescribed_coordinates','Position',  [100, 100, 667, 300]);

subplot(1,2,1);
plot(time,coord_data.hip_flex_r,'LineWidth',2)
ylim([0.0 40])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Hip Flexion (hip\_flex\_r)')
box off

subplot(1,2,2);
plot(time,coord_data.knee_flex_r,'LineWidth',2)
ylim([0.0 40])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Knee Flexion (knee\_flex\_r)')
box off

saveas(coord_fig,'../graphics/prescribed_coordinates.png')

%% Muscle File
muscle_input_sto_file = 'muscle_inputs.sto';

max_control = 0.75;
max_activation = 0.75;
max_force = 500;

msl_data.time = time;

control_points = [0,0,0,0,max_control];
activation_points = [0,0,0,0,max_activation];
force_points = [0,0,0,0,max_force];

smooth_control = interp1(time_points,control_points, time,'pchip');
smooth_activation = interp1(time_points,activation_points, time,'pchip');
smooth_force = interp1(time_points,force_points, time,'pchip');

msl_data.vasmed_r_control = smooth_control';

msl_data.vaslat_r_activation = smooth_activation';

msl_data.vasint_r_force = smooth_force';

msl_table = osimTableFromStruct(msl_data); %% Function distributed in OpenSim 4.0 resources

STOFileAdapter.write(msl_table,muscle_input_sto_file);

% Muscle input plots
msl_fig = figure('name','muscle_inputs','Position',  [100, 100, 1000, 300]);

subplot(1,3,1);
plot(time,msl_data.vasmed_r_control,'LineWidth',2)
ylim([0.0 1.0])
xlabel('Time [s]')
ylabel('Control')
title('Vastus Medialis (vasmed\_r)')
box off

subplot(1,3,2);
plot(time,msl_data.vaslat_r_activation,'LineWidth',2)
ylim([0.0 1.0])
xlabel('Time [s]')
ylabel('Activation')
title('Vastus Lateralis (vaslat\_r)')
box off

subplot(1,3,3);
plot(time,msl_data.vasint_r_force,'LineWidth',2)
ylim([0.0 100.0])
xlabel('Time [s]')
ylabel('Force [N]')
title('Vastus Intermedius (vasint\_r)')
box off

saveas(msl_fig,'../graphics/muscle_inputs.png')