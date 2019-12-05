%% Create Muscle Force, Activations, and Prescribed Coordinates files
%==========================================================================
import org.opensim.modeling.*
plugin = '../../../../jam_plugin.dll';
Model.LoadOpenSimLibrary(plugin);

msl_file = 'muscle_inputs.sto';
coord_file = 'quadriceps_load_prescribed_coordinates.sto';

time_step = 0.01;

settle_duration = 0.5;
flex_duration = 0.5;
act_duration = 1.0;
total_duration = settle_duration + flex_duration + act_duration;

settle_time = 0:time_step:settle_duration;
flex_time = settle_duration:time_step:flex_duration + settle_duration;
act_time = flex_duration + settle_duration: time_step : flex_duration + settle_duration + act_duration;
time = 0:time_step:total_duration;

nSettleSteps = floor(settle_duration/time_step);
nFlexSteps = floor(flex_duration/time_step);
nActSteps = floor(act_duration/time_step)+1;

sto = STOFileAdapter();

nSteps = length(time);


%% Muscle File
msl_data.time = time;
msl_data.vasint_r_control = linspace(0,1.0,nSteps)';
msl_data.vasmed_r_act = linspace(0,1.0,nSteps)';
msl_data.vaslat_r_frc = linspace(0,20.0,nSteps)';
table_labels = {'/forceset/vasint_r_control','/forceset/vasmed_r_act','/forceset/vaslat_r_frc'};
msl_table = osimTableFromStruct(msl_data,'table_labels',table_labels); %% Function distributed in OpenSim 4.0 resources

sto.write(msl_table,msl_file);

%% Coordinate File
coord_data.time = time;
max_flex = 30;
coord_data.knee_flex_r = [zeros(nSettleSteps,1); linspace(0,max_flex,nFlexSteps)'; ones(nActSteps,1)*max_flex];
table_labels = {'/jointset/knee_r/knee_flex_r'};
coord_table = osimTableFromStruct(coord_data,'table_labels',table_labels);
coord_table.addTableMetaDataString('inDegrees','yes');
sto.write(coord_table,coord_file);


