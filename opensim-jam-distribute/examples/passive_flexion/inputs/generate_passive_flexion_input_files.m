%% Create  Prescribed Coordinates file
%=====================================
import org.opensim.modeling.*

coord_file = 'knee_flexion.sto';

time_step = 0.01;

settle_duration = 1.0;
flex_duration = 3.0;

settle_time = 0:time_step:settle_duration;
flex_time = settle_duration:0.01:flex_duration + settle_duration;

time = 0:time_step:flex_duration + settle_duration;

nSettleSteps = floor(settle_duration/time_step);
nFlexSteps = floor(flex_duration/time_step);

sto = STOFileAdapter();

%% Coordinate File
coord_data.time = time;
coord_data.knee_flex_r = [zeros(nSettleSteps,1); linspace(0,90,nFlexSteps)'];
table_labels = {'/jointset/knee_r/knee_flex_r'};
coord_table = osimTableFromStruct(coord_data,'table_labels',table_labels);

sto.write(coord_table,coord_file);


