%% Plot Walking Simulation Results
%==========================================================================
close all;
import org.opensim.modeling.*

line_width = 2;
BW = 61*9.81;
%model = Model('../../../models/lenhart2015/lenhart2015.osim');

%% Plot Knee Kinematics
[values_data, values_labels, values_header] = read_opensim_mot('../results/comak/walking_values.sto');
time = values_data(:,1);

tf_coords = {...
    {'knee_flex_r';'knee\_flex\_r';'Flexion';'Angle[^o]'};...
    {'knee_add_r';'knee\_add\_r';'Adduction';'Angle[^o]'};...
    {'knee_rot_r';'knee\_rot\_r';'Internal Rotation';'Angle[^o]'};...
    {'knee_tx_r';'knee\_tx\_r';'Anterior Translation';'Translation [mm]'};...
    {'knee_ty_r';'knee\_ty\_r';'Superior Translation';'Translation [mm]'};...
    {'knee_tz_r';'knee\_tz\_r';'Lateral Translation';'Translation [mm]'}};

tf_kin_fig = figure('name','Tibiofemoral Kinematics','Position',[100,100,1000,600]);
hold on;

for i = 1:length(tf_coords)
    ind = find(contains(values_labels,tf_coords{i}{1}));
    if(contains(tf_coords{i}{4},'Translation'))
        data = values_data(:,ind)*1000;
    else
        data = values_data(:,ind);
    end
        
    subplot(2,3,i);
    plot(time,data,'LineWidth',line_width)
    title([tf_coords{i}{3} ' (' tf_coords{i}{2} ')'])
    xlabel('Time [s]')
    ylabel(tf_coords{i}{4})
end
    
saveas(tf_kin_fig,'../graphics/walking_tibiofemoral_kinematics.png')

pf_coords = {...
    {'pf_flex_r';'pf\_flex\_r';'Flexion';'Angle[^o]'};...
    {'pf_rot_r';'pf\_rot\_r';'Rotation';'Angle[^o]'};...
    {'pf_tilt_r';'pf\_tilt\_r';'Tilt';'Angle[^o]'};...
    {'pf_tx_r';'pf\_tx\_r';'Anterior Translation';'Translation [mm]'};...
    {'pf_ty_r';'pf\_ty\_r';'Superior Translation';'Translation [mm]'};...
    {'pf_tz_r';'pf\_tz\_r';'Lateral Translation';'Translation [mm]'}};

pf_kin_fig = figure('name','Patellofemoral Kinematics','Position',[100,100,1000,600]);
hold on;
for i = 1:length(pf_coords)
    ind = find(contains(values_labels,pf_coords{i}{1}));
    if(contains(pf_coords{i}{4},'Translation'))
        data = values_data(:,ind)*1000;
    else
        data = values_data(:,ind);
    end
    
    subplot(2,3,i);
    plot(time,data,'LineWidth',line_width)
    title([pf_coords{i}{3} ' (' pf_coords{i}{2} ')'])
    xlabel('Time [s]')
    ylabel(pf_coords{i}{4})
end

saveas(pf_kin_fig,'../graphics/walking_patellofemoral_kinematics.png')

%% Plot Contact Forces
[forces_data, forces_labels, forces_header] = read_opensim_mot('../results/joint-mechanics/walking_ForceReporter_forces.sto');
[forces_weights_data, forces_weights_labels, forces_weights_header] = read_opensim_mot('../results/joint-mechanics_muscle_weights/walking_muscle_weights_ForceReporter_forces.sto');

ind = find(contains(forces_labels,'tf_contact.casting.total.contact_force_y'));
ind_weights = find(contains(forces_weights_labels,'tf_contact.casting.total.contact_force_y'));

forces_time = forces_data(:,1);
forces_weights_time = forces_weights_data(:,1);

figure('name','Tibiofemoral Contact Forces')
hold on;
plot(forces_time,forces_data(:,ind)/BW,'LineWidth',line_width)
plot(forces_time,forces_weights_data(:,ind_weights)/BW,'LineWidth',line_width)
legend('no weights','muscle weights')


%% Plot GRF

%% Plot Muscle Activation
% [act_data, act_labels, act_header] = read_opensim_mot('../results/comak/walking_activations.sto');
% 
% time = act_data(:,1);
% 
% figure('name','Muscle Activation - All')
% hold on;
% numMuscles = model.getMuscles().getSize();
% msl_names = cell(numMuscles,1);
% 
% for m=0:numMuscles-1
%    msl_path = model.getMuscles().get(m).getAbsolutePathString();
%    
%    activation = act_data(:,strcmp(act_labels,msl_path));
%    plot(states_time,activation,'LineWidth',line_width)
%    
%    msl_names{m+1} = char(model.getMuscles().get(m).getName());
% end
% legend(msl_names)
% title('Muscle Activations')
% xlabel('Time [s]')
% ylabel('Activation')
% ylim([0 1]);
% 
% 
% if(false)
%     numMuscles = model.getMuscles().getSize();
%     msl_names = cell(numMuscles,1);
% 
%     for m=0:numMuscles-1
%         msl_names{m+1} = char(model.getMuscles().get(m).getName());
%        figure('name',msl_names{m+1})
%        msl_path = model.getMuscles().get(m).getAbsolutePathString();
% 
%        activation = states_data(:,strcmp(states_labels,msl_path));
%        plot(states_time,activation)
%            title(msl_names{m+1})
%         xlabel('Time [s]')
%         ylabel('Activation')
%         ylim([0 1]);
%     end
% 
% end
