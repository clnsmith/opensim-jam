%% Create  Prescribed Coordinates file
%=====================================
import org.opensim.modeling.*

coord_file = 'prescribed_coordinates.sto';

%% Simulation Time
%Simulation consists of twp phases:
%settle: allow knee to settle into equilbrium 
%flex: prescribe the tibiofemoral flexion

time_step = 0.01;

settle_duration = 0.5;
flex_duration = 2.0;

settle_time = 0 : time_step : settle_duration;
flex_time = settle_duration + time_step : time_step : flex_duration + settle_duration;

time = [settle_time, flex_time];

nSettleSteps = length(settle_time);
nFlexSteps = length(flex_time);

sto = STOFileAdapter();

%% Prescribed Coordinate File
coord_data.time = time;
coord_data.pelvis_tilt = ones(nSettleSteps+nFlexSteps,1)*90;
coord_data.knee_flex_r = [zeros(nSettleSteps,1); linspace(0,90,nFlexSteps)'];
coord_table = osimTableFromStruct(coord_data);

sto.write(coord_table,coord_file);

% Prescribed coordinates plot
coord_fig = figure('name','prescribed_coordinates','Position',  [100, 100, 667, 300]);

subplot(1,2,1);
plot(time,coord_data.pelvis_tilt,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Pelvis Tilt (pelvis\_tilt\_r)')
box off

subplot(1,2,2);
plot(time,coord_data.knee_flex_r,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Knee Flexion (knee\_flex\_r)')
box off

saveas(coord_fig,'../graphics/prescribed_coordinates.png')