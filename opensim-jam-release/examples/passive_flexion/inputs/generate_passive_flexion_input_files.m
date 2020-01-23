%% Create  Prescribed Coordinates file
%=====================================
import org.opensim.modeling.*

coord_file = 'passive_flexion_prescribed_coordinates.sto';

%% Simulation Time
%Simulation consists of twp phases:
%settle: allow knee to settle into equilbrium 
%flex: prescribe the tibiofemoral flexion

time_step = 0.01;

settle_duration = 0.5;
flex_duration = 2.0;

settle_time = 0 : time_step : settle_duration;
flex_time = settle_duration : time_step : flex_duration + settle_duration;

time = [settle_time, flex_time];

nSettleSteps = length(settle_time);
nFlexSteps = length(flex_time);

sto = STOFileAdapter();

%% Prescribed Coordinate File
coord_data.time = time;
coord_data.pelvis_tilt = ones(nSettleSteps+nFlexSteps,1)*90;
coord_data.knee_flex_r = [zeros(nSettleSteps,1); linspace(0,90,nFlexSteps)'];
coord_table = osimTableFromStruct(coord_data);

sto.write(coord_table,coord_file);

% Prescribed coordinates plot
coord_fig = figure('name','prescribed_coordinates','Position',  [100, 100, 667, 300]);

subplot(1,2,1);
plot(time,coord_data.pelvis_tilt,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Pelvis Tilt (pelvis\_tilt\_r)')
box off

subplot(1,2,2);
plot(time,coord_data.knee_flex_r,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Knee Flexion (knee\_flex\_r)')
box off

saveas(coord_fig,'../graphics/prescribed_coordinates.png')

%% Model Files
plugin_file = '../../../bin/jam_plugin.dll';
opensimCommon.LoadOpenSimLibrary(plugin_file)

lenhart_model_file = '../../../models/lenhart2015/lenhart2015.osim';

alta_model_file = '../../../models/lenhart2015/lenhart2015_alta.osim';
normal_model_file = '../../../models/lenhart2015/lenhart2015_alta_PTA_normal.osim';
baja_model_file = '../../../models/lenhart2015/lenhart2015_alta_PTA_baja.osim';

length_factor = 0.15;
PH = PropertyHelper();




% Alta Model
%------------
% Change the PT reference strain to simulate patella alta
alta_model = Model(lenhart_model_file);

for i = 0:alta_model.getForceSet.getSize()-1
    force = alta_model.getForceSet.get(i);
    if(contains(char(force.getName()),'PT'))
        slack_length = PH.getValueDouble(force.getPropertyByName('slack_length'));
        slack_length = slack_length * length_factor;
        PH.setValueDouble(slack_length,force.getPropertyByName('slack_length'));        
    end
end

alta_model.initSystem();
alta_model.print(alta_model_file);
alta_model.initSystem();
alta_model.print(alta_model_file);

% Normal Model
%------------
% Increase the PT reference strain to simulate patella alta
% Perform Patellar Tendon Advancement to return patella to normal position
normal_model = Model(lenhart_model_file);

for i = 0:normal_model.getForceSet.getSize()-1
    force = normal_model.getForceSet.get(i);
    if(contains(char(force.getName()),'PT'))
        
        slack_length = PH.getValueDouble(force.getPropertyByName('slack_length'));
        new_slack_length = slack_length *(1 + length_factor);
        PH.setValueDouble(new_slack_length,force.getPropertyByName('slack_length'));
        
        geo_path = GeometryPath.safeDownCast(force.getPropertyByName('GeometryPath').getValueAsObject());
        path_point_set = geo_path.getPathPointSet();
        for p = 0:path_point_set.getSize()-1
            path_point = path_point_set.get(p);
            if (contains(char(path_point.getSocket('parent_frame').getConnecteePath()),'tibia_proximal_r'))
                yPos = PH.getValueVec3(path_point.getPropertyByName('location'),1);
                new_yPos = yPos - slack_length * length_factor;
                PH.setValueVec3(new_yPos,path_point.getPropertyByName('location'),1);
            end
        end        
    end
end

normal_model.initSystem();
normal_model.print(normal_model_file);

% Baja Model
%------------
% Increase the PT reference strain to simulate patella alta
% Perform Patellar Tendon Advancement to 'over correct' patella to baja position
baja_model = Model(lenhart_model_file);

for i = 0:baja_model.getForceSet.getSize()-1
    force = baja_model.getForceSet.get(i);
    if(contains(char(force.getName()),'PT'))
        
        slack_length = PH.getValueDouble(force.getPropertyByName('slack_length'));
        new_slack_length = slack_length *(1 + length_factor);
        PH.setValueDouble(new_slack_length,force.getPropertyByName('slack_length'));
        
        geo_path = GeometryPath.safeDownCast(force.getPropertyByName('GeometryPath').getValueAsObject());
        path_point_set = geo_path.getPathPointSet();
        for p = 0:path_point_set.getSize()-1
            path_point = path_point_set.get(p);
            if (contains(char(path_point.getSocket('parent_frame').getConnecteePath()),'tibia_proximal_r'))
                yPos = PH.getValueVec3(path_point.getPropertyByName('location'),1);
                new_yPos = yPos - 2 * slack_length * length_factor;
                PH.setValueVec3(new_yPos,path_point.getPropertyByName('location'),1);
            end
        end        
    end
end

baja_model.initSystem();
baja_model.print(baja_model_file);
