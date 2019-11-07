%% Create Muscle Force, Activations, and Prescribed Coordinates files
%==========================================================================
import org.opensim.modeling.*
plugin = 'C:\Users\csmith\github\wisco_opensim\install\plugin\WISCO_Plugin.dll';
Model.LoadOpenSimLibrary(plugin);

msl_file = 'C:\Users\csmith\github\wisco_opensim\source\examples\cmd\forsim\msl_input.sto';
coord_file = 'C:\Users\csmith\github\wisco_opensim\source\examples\cmd\forsim\coord_input.sto';
template_setting_file = 'C:\Users\csmith\github\wisco_opensim\source\examples\cmd\forsim\forsim_settings_template.xml';
setting_file = 'C:\Users\csmith\github\wisco_opensim\source\examples\cmd\forsim\forsim_settings.xml';
time = 0:0.02:4;

nSteps = length(time);

sto = STOFileAdapter();
%% Muscle File
msl_data.time = time;
msl_data.vasmed_r_act = linspace(0,1.0,nSteps)';
msl_data.vaslat_r_frc = linspace(0,20.0,nSteps)';
table_labels = {'/forceset/vasmed_r_act','/forceset/vaslat_r_frc'};
msl_table = osimTableFromStruct(msl_data,'table_labels',table_labels); %% Function distributed in OpenSim 4.0 resources

sto.write(msl_table,msl_file);

%% Coordinate File
coord_data.time = time;
coord_data.knee_flex_r = linspace(0,90,nSteps)';
table_labels = {'/jointset/knee_r/knee_flex_r'};
coord_table = osimTableFromStruct(coord_data,'table_labels',table_labels);

sto.write(coord_table,coord_file);


