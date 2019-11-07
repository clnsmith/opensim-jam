%% Create  Prescribed Coordinates file
%=====================================
import org.opensim.modeling.*


coord_file = 'C:\Users\csmith\github\wisco_opensim\source\examples\cmd\forsim\knee_flexion.sto';

time = 0:0.02:4;

nSteps = length(time);

sto = STOFileAdapter();

%% Coordinate File
coord_data.time = time;
coord_data.knee_flex_r = linspace(0,90,nSteps)';
table_labels = {'/jointset/knee_r/knee_flex_r'};
coord_table = osimTableFromStruct(coord_data,'table_labels',table_labels);

sto.write(coord_table,coord_file);


