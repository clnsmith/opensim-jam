%% Plot Walking Simulation Results
%==========================================================================
close all;
import org.opensim.modeling.*

line_width = 2;
BW = 61*9.81;
model = Model('../../../models/lenhart2015/lenhart2015.osim');
%% Force Reporter
forces_table = TimeSeriesTable('../results/joint-mechanics/walking_ForceReporter_forces.sto');
forces = osimTableToStruct(forces_table);

[states_data, states_labels, states_header] = read_opensim_mot('../results/comak/walking_activation.sto');
states_time = states_data(:,1);

% forces_table = TimeSeriesTable('../results/joint-mechanics/walking_ForceReporter_forces.sto');
% forces = osimTableToStruct(forces_table);
% 
% [states_data, states_labels, states_header] = read_opensim_mot('../results/comak/walking_activation.sto');
% states_time = states_data(:,1);

%% Plot Contact Forces
figure('name','Tibiofemoral Contact Forces')
hold on
plot(forces.time,forces.tf_contact_casting_total_contact_force_x/BW,'LineWidth',line_width)
plot(forces.time,-forces.tf_contact_casting_total_contact_force_y/BW,'LineWidth',line_width)
plot(forces.time,forces.tf_contact_casting_total_contact_force_z/BW,'LineWidth',line_width)
legend('Fx','Fy','Fz')

figure('name','Patellofemoral Contact Forces')
hold on
plot(forces.time,forces.pf_contact_casting_total_contact_force_x/BW,'LineWidth',line_width)
plot(forces.time,forces.pf_contact_casting_total_contact_force_y/BW,'LineWidth',line_width)
plot(forces.time,forces.pf_contact_casting_total_contact_force_z/BW,'LineWidth',line_width)
legend('Fx','Fy','Fz')

%% Plot Muscle Activation
figure('name','Muscle Activation - All')
hold on;
numMuscles = model.getMuscles().getSize();
msl_names = cell(numMuscles,1);

for m=0:numMuscles-1
   msl_path = model.getMuscles().get(m).getAbsolutePathString();
   
   activation = states_data(:,strcmp(states_labels,msl_path));
   plot(states_time,activation,'LineWidth',line_width)
   
   msl_names{m+1} = char(model.getMuscles().get(m).getName());
end
legend(msl_names)
title('Muscle Activations')
xlabel('Time [s]')
ylabel('Activation')
ylim([0 1]);


if(true)
    numMuscles = model.getMuscles().getSize();
    msl_names = cell(numMuscles,1);

    for m=0:numMuscles-1
        msl_names{m+1} = char(model.getMuscles().get(m).getName());
       figure('name',msl_names{m+1})
       msl_path = model.getMuscles().get(m).getAbsolutePathString();

       activation = states_data(:,strcmp(states_labels,msl_path));
       plot(states_time,activation)
           title(msl_names{m+1})
        xlabel('Time [s]')
        ylabel('Activation')
        ylim([0 1]);
    end

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