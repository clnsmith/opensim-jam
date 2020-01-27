%% Plot Anterior Laxity Example Results
%==========================================================================
close all;
import org.opensim.modeling.*

line_width = 2;
BW = 61*9.81;
model = Model('../../../models/lenhart2015/lenhart2015.osim');

%% Force Reporter
% forces_table = TimeSeriesTable('../results/joint-mechanics/walking_ForceReporter_forces.sto');
% forces = osimTableToStruct(forces_table);

[healthy_states_data, healthy_states_labels, healthy_states_header] = read_opensim_mot('../results/forsim/anterior_laxity_states.sto');
healthy_states_time = healthy_states_data(:,1);

[acld_states_data, acld_states_labels, acld_states_header] = read_opensim_mot('../results/forsim_acld/anterior_laxity_acld_states.sto');
acld_states_time = acld_states_data(:,1);

% forces_table = TimeSeriesTable('../results/joint-mechanics/walking_ForceReporter_forces.sto');
% forces = osimTableToStruct(forces_table);
% 
% [states_data, states_labels, states_header] = read_opensim_mot('../results/comak/walking_activation.sto');
% states_time = states_data(:,1);

%% Plot Knee Kinematics
tf_coords = {...
    '/jointset/knee_r/knee_flex_r/value';...
    '/jointset/knee_r/knee_add_r/value';...
    '/jointset/knee_r/knee_rot_r/value';...
    '/jointset/knee_r/knee_tx_r/value';...
    '/jointset/knee_r/knee_ty_r/value';...
    '/jointset/knee_r/knee_tz_r/value';...
    };


figure('name','Tibiofemoral Kinematics')
hold on;
for i = 1:length(tf_coords)
    ind = find(contains(healthy_states_labels,tf_coords{i}));
    subplot(3,2,i);hold on;
    plot(healthy_states_time,healthy_states_data(:,ind),'b','LineWidth',line_width)
    plot(acld_states_time,acld_states_data(:,ind),'r-','LineWidth',line_width)
    title(tf_coords{i})
end
    


