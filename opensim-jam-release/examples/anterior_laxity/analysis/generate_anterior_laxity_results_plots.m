%% Plot Anterior Laxity Example Results
%==========================================================================
close all;
import org.opensim.modeling.*

line_width = 2;

[healthy_states_data, healthy_states_labels, healthy_states_header] = read_opensim_mot('../results/forsim/anterior_laxity_states.sto');
healthy_states_time = healthy_states_data(:,1);

[acld_states_data, acld_states_labels, acld_states_header] = read_opensim_mot('../results/forsim_acld/anterior_laxity_acld_states.sto');
acld_states_time = acld_states_data(:,1);


%% Plot All Knee Kinematics
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
legend('healthy','acl_deficient')

%% Plot Anterior Translation
anterior_translation = '/jointset/knee_r/knee_tx_r/value';

ap_fig = figure('name','Anterior Translation','Position',  [100, 100, 333, 300]);
hold on;

healthy_ap_ind = find(contains(healthy_states_labels,anterior_translation));
acld_ap_ind = find(contains(acld_states_labels,anterior_translation));

healthy_ap = healthy_states_data(:,healthy_ap_ind)*1000; % convert to mm
acld_ap = acld_states_data(:,acld_ap_ind)*1000; % convert to mm

hold on;
plot(healthy_states_time,healthy_ap,'b','LineWidth',line_width)
plot(acld_states_time,acld_ap,'r-','LineWidth',line_width)
title('Anterior Translation (knee\_tx\_r)')
xlabel('Time [s]')
ylabel('Translation [mm]')
legend('healthy','acl deficient')

saveas(ap_fig,'../graphics/anterior_translation_healthy_ACLd.png')

%% Plot ACL Force and Strain
[healthy_forces_data, healthy_forces_labels, healthy_forces_header] = read_opensim_mot('../results/forsim/anterior_laxity_ForceReporter_forces.sto');
healthy_forces_time = healthy_forces_data(:,1);

acl_frc_ind = find(and(contains(healthy_forces_labels,'ACL'),endsWith(healthy_forces_labels,'force_total')));
acl_strain_ind = find(and(contains(healthy_forces_labels,'ACL'),endsWith(healthy_forces_labels,'strain')));

ACL_fig = figure('name','ACL Strain and Force','Position',  [100, 100, 1000, 500]);

subplot(1,2,1);hold on;
for i = 1:length(acl_frc_ind)    
    plot(healthy_forces_time,healthy_forces_data(:,acl_frc_ind(i)),'LineWidth',line_width);
    
end
title('ACL Force')
xlabel('Time [s]')
ylabel('Force [N]')


subplot(1,2,2);hold on;
for i = 1:length(acl_strain_ind)    
    plot(healthy_forces_time,healthy_forces_data(:,acl_strain_ind)*100,'LineWidth',line_width);
    
end
title('ACL Strain')
xlabel('Time [s]')
ylabel('Strain [%]')

for i =1:length(acl_strain_ind)
    split_name = strsplit(healthy_forces_labels{acl_strain_ind(i)},'.');
    names{i} = split_name{1};
end
legend(names,'Location','southeast')

saveas(ACL_fig,'../graphics/ACL_force_strain.png')