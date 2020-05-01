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
[forces_contact_data, forces_contact_labels, forces_contact_header] = read_opensim_mot('../results/joint-mechanics_contact_energy/walking_contact_energy_ForceReporter_forces.sto');

ind = find(contains(forces_labels,'tf_contact.casting.total.contact_force_y'));
ind_weights = find(contains(forces_weights_labels,'tf_contact.casting.total.contact_force_y'));
ind_contact = find(contains(forces_contact_labels,'tf_contact.casting.total.contact_force_y'));
forces_time = forces_data(:,1);
forces_weights_time = forces_weights_data(:,1);
forces_contact_time = forces_contact_data(:,1);

kcf_fig = figure('name','Tibiofemoral Contact Forces');
hold on;
plot(forces_time,forces_data(:,ind)/BW,'LineWidth',line_width)
plot(forces_weights_time,forces_weights_data(:,ind_weights)/BW,'LineWidth',line_width)
plot(forces_contact_time,forces_contact_data(:,ind_weights)/BW,'LineWidth',line_width)
legend('no weights','muscle weights','contact energy','Location','southeast')

saveas(kcf_fig,'../graphics/walking_knee_contact_force.png')

%% Plot Muscle Activation
[act_data, act_labels, act_header] = read_opensim_mot('../results/comak/walking_activation.sto');
[act_weights_data, act_weights_labels, act_weights_header] = read_opensim_mot('../results/comak_muscle_weights/walking_muscle_weights_activation.sto');
[act_contact_data, act_contact_labels, act_contact_header] = read_opensim_mot('../results/comak_contact_energy/walking_contact_energy_activation.sto');

act_time = act_data(:,1);
act_weights_time = act_weights_data(:,1);
act_contact_time = act_contact_data(:,1);

msls = {'soleus','gasmed_r','recfem_r','semimem_r','glmed1_r','glmin1_r'};
msl_names = {'soleus','gasmed','recfem','semimem','glmed1','glmin1'};

act_fig = figure('name','Muscle Activation - All','Position',[100 100 1600 400]);

for i = 1:length(msls)
    ind = find(contains(act_labels,msls{i}));
    ind_weights = find(contains(act_weights_labels,msls{i}));
    ind_contact = find(contains(act_contact_labels,msls{i}));
    
    subplot(2,3,i);hold on;
    plot(act_time,act_data(:,ind),'LineWidth',line_width);
    plot(act_weights_time,act_weights_data(:,ind_weights),'LineWidth',line_width);
    plot(act_contact_time,act_contact_data(:,ind_contact),'LineWidth',line_width);
    
    title(msl_names{i})
    xlabel('Time [s]')
    ylabel('Activation')
end
legend('no weights','muscle weights','contact energy')
saveas(act_fig,'../graphics/walking_muscle_activations.png')
