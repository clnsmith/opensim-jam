%% Generate Input Files for Anterior Laxity Example
%==========================================================================
%Author: Colin Smith
%
%
%==========================================================================
import org.opensim.modeling.*

force_magnitude = 89; % 89N Common value in KT-1000 device
force_point_height = -0.1; %Apply at the tibial tuberosity height

%% Simulation Time
%Simulation consists of three phases:
%flex: hip and knee flexion
%settle: allow knee to settle into equilbrium 
%force: ramp up the anterior force

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

max_hip_flex = 45;
max_knee_flex = 90;

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
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Hip Flexion (hip\_flex\_r)')
box off

subplot(1,2,2);
plot(time,coord_data.knee_flex_r,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Knee Flexion (knee\_flex\_r)')
box off

saveas(coord_fig,'../graphics/prescribed_coordinates.png')

%% External Loads Files
%.sto file
%---------
external_loads_sto_file = 'external_loads.sto';


force_data.time = time;
force_data.force_vx = ...
    [zeros(num_flex_steps + num_settle_steps,1);...
     linspace(0,force_magnitude,num_force_steps)'];
force_data.force_vy = zeros(num_steps,1);
force_data.force_vz = zeros(num_steps,1);
force_data.force_px = zeros(num_steps,1);
force_data.force_py = ones(num_steps,1)*force_point_height;
force_data.force_pz = zeros(num_steps,1);
force_data.torque_x = zeros(num_steps,1);
force_data.torque_y = zeros(num_steps,1);
force_data.torque_z = zeros(num_steps,1);

force_table = osimTableFromStruct(force_data); %% Function distributed in OpenSim 4.0 resources
STOFileAdapter.write(force_table,external_loads_sto_file);

% external loads plot
ext_loads_fig = figure('name','external_loads','Position',  [100, 100, 333, 300]);

plot(time,force_data.force_vx,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Anterior Force [N]')
title('External Loads on the Tibia')
box off

saveas(ext_loads_fig,'../graphics/external_loads.png')

% .xml file
%----------
external_loads_xml_file = 'external_loads.xml';

ext_force = ExternalForce();
ext_force.setName('AnteriorForce');
ext_force.set_applied_to_body('tibia_proximal_r');
ext_force.set_force_expressed_in_body('tibia_proximal_r');
ext_force.set_point_expressed_in_body('tibia_proximal_r');
ext_force.set_force_identifier('force_v');
ext_force.set_point_identifier('force_p');
ext_force.set_torque_identifier('torque_');

ext_loads = ExternalLoads();
ext_loads.setDataFileName(external_loads_sto_file);
ext_loads.adoptAndAppend(ext_force);
ext_loads.print(external_loads_xml_file );



