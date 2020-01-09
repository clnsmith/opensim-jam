%% Plot Walking Simulation Results
%==========================================================================
close all;
import org.opensim.modeling.*

line_width = 2;
BW = 61*9.81;
%% Force Reporter
forces_table = TimeSeriesTable('../results/joint-mechanics/walking_ForceReporter_forces.sto');
forces = osimTableToStruct(forces_table);

% states_table = TimeSeriesTable('../results/comak/walking_states.sto');
% states = osimTableToStruct(states_table);

[states_data, states_labels, states_header] = read_opensim_mot('../results/comak/walking_states.sto');
%% Plot Contact Forces
figure('name','Tibiofemoral Contact Forces')
hold on
plot(forces.time,forces.tf_contact_casting_contact_force_x/BW,'LineWidth',line_width)
plot(forces.time,-forces.tf_contact_casting_contact_force_y/BW,'LineWidth',line_width)
plot(forces.time,forces.tf_contact_casting_contact_force_z/BW,'LineWidth',line_width)
legend('Fx','Fy','Fz')

figure('name','Patellofemoral Contact Forces')
hold on
plot(forces.time,forces.pf_contact_casting_contact_force_x/BW,'LineWidth',line_width)
plot(forces.time,forces.pf_contact_casting_contact_force_y/BW,'LineWidth',line_width)
plot(forces.time,forces.pf_contact_casting_contact_force_z/BW,'LineWidth',line_width)
legend('Fx','Fy','Fz')

%% Plot Muscle Activation
figure('name','Muscle Activation - All')
hold on;
for m = 1:nMuscles 
   
end

%% Plot Knee Kinematics
tf_coords = {...
    '/jointset/knee_r/knee_flex_r/value';...
    '/jointset/knee_r/knee_add_r/value';...
    '/jointset/knee_r/knee_rot_r/value';...
    '/jointset/knee_r/knee_tx_r/value';...
    '/jointset/knee_r/knee_ty_r/value';...
    '/jointset/knee_r/knee_tz_r/value';...
    };

pf_coords = {...
    '/jointset/pf_r/knee_flex_r/value';...
    '/jointset/pf_r/knee_add_r/value';...
    '/jointset/pf_r/knee_rot_r/value';...
    '/jointset/pf_r/knee_tx_r/value';...
    '/jointset/pf_r/knee_ty_r/value';...
    '/jointset/pf_r/knee_tz_r/value';...
    };

figure('name','Tibiofemoral Kinematics')
hold on;
for i = 1:length(tf_coords)
    ind = find(contains(states_labels,tf_coords{i}));
    subplot(3,2,i);
    plot(states_data(:,1),states_data(:,ind),'LineWidth',line_width)
    title(tf_coords{i})
end
    
figure('name','Patellofemoral Kinematics')
hold on;
for i = 1:length(pf_coords)
    ind = find(contains(states_labels,pf_coords{i}));
    subplot(3,2,i);
    plot(states_data(:,1),states_data(:,ind),'LineWidth',line_width)
    title(pf_coords{i})
end

% figure('name','Tibiofemoral Kinematics')
% hold on;
% for i = 1:length(tf_coords)
%     plot(states.time,states.(tf_coords{i}))
% end
%     
% figure('name','Patellofemoral Kinematics')
% hold on;
% for i = 1:length(pf_coords)
%     plot(states.time,states.(pf_coords{i}))
% end