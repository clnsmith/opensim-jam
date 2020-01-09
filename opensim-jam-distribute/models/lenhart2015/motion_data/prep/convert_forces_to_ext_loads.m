%% Convert forces file to ext_loads
%==========================================================================

import org.opensim.modeling.*

forces_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\prep\overground_17.forces';
ext_load_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\overground_17_ext_loads.xml';
raw_mot_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\prep\overground_17_grf_raw.mot';
mot_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\overground_17_grf.mot';
mot_file_base = 'overground_17_grf.mot';
[fp,time,f,~,nf,fnames,file,inpath]=load_forces(forces_file,'');
nf=3;%Only prints force plates 1,2,3 because 4 and 5 don't contain data
%Set column Labels
col_labels = StdVectorString;


in_label = {'F','','M'};
out_label = {'v','p',''};
out_type = {'force','force','torque'};
in_dir = {'X','Z','Y'};
out_dir = {'x','y','z'}; %Rotate coordinate systems



data = zeros(length(time),nf*9);
c=1;
for i = 1:nf
   for j = 1:3
       for k = 1:3
           in_name = [in_label{j} in_dir{k} int2str(i)];
           
           ind = find(strcmp(fnames,in_name));
           if(~isempty(ind))
               data(:,c) = fp(:,ind);
               
           end
           %%Convert point to meters
           if(strcmp(in_label{j},''))
                data(:,c) = data(:,c)/1000;
           end
           %%Convert Torque to nm
           if(strcmp(in_label{j},'M'))
                data(:,c) = data(:,c)/1000;
           end
           %%Convert Direction
           if(strcmp(in_dir{k},'Y'))
               data(:,c) = data(:,c)*-1;
           end
           
           out_name = ['ground_' out_type{j}  '_' int2str(i) '_' out_label{j} out_dir{k}];
           col_labels.add(out_name)
           c=c+1;
       end
   end
end


table = TimeSeriesTable();
table.setColumnLabels(col_labels);

%Set Data
[m,n] = size(data);

%data(:,4:6) = data(:,4:6)/1000;
%data(:,13:15) = data(:,13:15)/1000;
%data(:,22:24) = data(:,22:24)/1000;

row_data = RowVector(n,0.0);

for i = 1:m
    for j = 1:n 
        row_data.set(j-1,data(i,j));
    end
    table.appendRow(time(i),row_data);
end

table.addTableMetaDataString('header','Y9_100_1_ground_reaction');
table.addTableMetaDataString('nColumns',num2str(n+1));
table.addTableMetaDataString('nRows',num2str(m));
table.addTableMetaDataString('inDegrees','no');

sto_fa = STOFileAdapter();
sto_fa.write(table,raw_mot_file);

%% Filter Ground Reactions
vert_frc_threshold = 10;

[data, labels, header] = read_opensim_mot(raw_mot_file);

%% Filter Data

Fe=1/(data(2,1)-data(1,1)); %Sampling frequency
Fc=25; % Cut-off frequency
N=4; % Filter Order
[B, A] = butter(N,Fc*2/Fe, 'low');

filt_data = zeros(size(data));
filt_data(:,1) = data(:,1);

proc_data = zeros(size(data));
proc_data(:,1) = data(:,1);

vert_frc_ind = [];
nPlates = 0;
for i = 2:size(data,2)
    split_label = strsplit(labels{i},'_');
    comp = split_label{end};
    if(contains(comp,'vy'))
        nPlates = nPlates + 1;
        vert_frc_ind(nPlates) = i;
        
        vert_frc_data(:,nPlates) = data(:,i);
    end
end



for i = 2:size(data,2)
    split_label = strsplit(labels{i},'_');
    comp = split_label{end};
    frc_plate_num = str2double(split_label{end-1});

    start = -1;
    stop = -1;    
    for t = 1:size(data,1)
        if(vert_frc_data(t,frc_plate_num) > vert_frc_threshold)
            proc_data(t,i) = data(t,i);
            
            if(start == -1)
                start = t;                
            end
            if(stop == -2)
                stop = -1;
            end
        else
            proc_data(t,i) = 0;
            
            if(start ~= -1 && stop == -1)
                stop =-2;
            elseif(start ~= -1 && stop == -2)
                stop = t-2;
            end
        end     
    end 
    if(start > -1)        
        if(contains(comp,'p'))
            pad_size =Fe/2;
            pad_data = padarray(proc_data(start:stop,i),pad_size,'replicate','both');
            filt_pad_data = filtfilt(B,A,pad_data);

            filt_data(start:stop,i) = filt_pad_data(pad_size+1:end-pad_size);
        else
            pad_size =100;
            filt_raw_data = filtfilt(B,A,data(start-pad_size:stop+pad_size,i));
            filt_data(start:stop,i) = filt_raw_data(pad_size+1:end-pad_size);
        end
    end
end

for i = 2:size(data,2)
    split_label = strsplit(labels{i},'_');
    comp = split_label{end};
    frc_plate_num = split_label{end-1};
    
    if(str2double(frc_plate_num) < 4)
        figure('name',labels{i});hold on;
        plot(data(:,1),data(:,i),'r');
        plot(filt_data(:,1),filt_data(:,i),'b');
    end
    
end

write_opensim_mot(mot_file,filt_data,labels,header)

%% External Loads file
el = ExternalLoads();
el.setDataFileName(mot_file_base);
body = {'calcn_l','calcn_r','calcn_l'};

for i = 1:nf
    ef = ExternalForce();
    ef.setName(['FP' num2str(i)]);
    ef.set_applied_to_body(body{i});
    ef.set_force_expressed_in_body('ground');
    ef.set_point_expressed_in_body('ground');
    ef.set_force_identifier(['ground_force_' num2str(i) '_v']);
    ef.set_point_identifier(['ground_force_' num2str(i) '_p']);
    ef.set_torque_identifier(['ground_torque_' num2str(i) '_']);
    el.adoptAndAppend(ef);    
end

el.print(ext_load_file);