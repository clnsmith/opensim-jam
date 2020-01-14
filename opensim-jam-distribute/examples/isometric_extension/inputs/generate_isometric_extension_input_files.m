%% Generate Input Files For Isometric Extension Example
%==========================================================================
% Author: Colin Smith
%
%
%==========================================================================
import org.opensim.modeling.*

%% Simulation Time
%Simulation consists of three phases:
%flex: hip and knee flexion
%settle: allow knee to settle into equilbrium 
%force: ramp up the muscle forces

time_step = 0.01;

flex_duration = 1.0;
settle_duration = 0.5;
force_duration = 1.0;

flex_time = 0 : time_step : flex_duration;
settle_time = flex_duration+time_step : time_step : flex_duration+settle_duration;
force_time = flex_duration+settle_duration+time_step : time_step : flex_duration+settle_duration+force_duration;
time = [flex_time,settle_time,force_time];

num_flex_steps = length(flex_time);
num_settle_steps = length(settle_time);
num_force_steps = length(force_time);
num_steps = length(time);


%% Prescribed Coordinates File
prescribed_coordinate_file = 'prescribed_coordinates.sto';

max_hip_flex = 30;
max_knee_flex = 30;


coord_data.time = time;
coord_data.hip_flex_r = [...
    linspace(0,max_hip_flex,num_flex_steps)';...
    ones(num_settle_steps+num_force_steps,1)*max_hip_flex];

coord_data.knee_flex_r = [...
    linspace(0,max_knee_flex,num_flex_steps)';...
    ones(num_settle_steps+num_force_steps,1)*max_knee_flex];


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

max_control = 0.5;
max_activation = 0.5;
max_force = 100;

msl_data.time = time;

msl_data.vasmed_r_control = [
    zeros(num_flex_steps+num_settle_steps,1);...
    linspace(0,max_control,num_force_steps)'
    ];

msl_data.vaslat_r_activation = [
    zeros(num_flex_steps+num_settle_steps,1);...
    linspace(0,max_activation,num_force_steps)'
    ];

msl_data.vasint_r_force = [
    zeros(num_flex_steps+num_settle_steps,1);...
    linspace(0,max_force,num_force_steps)'
    ];

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