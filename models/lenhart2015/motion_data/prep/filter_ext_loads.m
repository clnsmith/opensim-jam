%% Filter External Loads
%==========================================================================
close all;
in_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\overground_17_grf.mot';
out_file = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\overground_17_grf_filtered.mot';

vert_frc_threshold = 15;

[data, labels, header] = read_opensim_mot(in_file);

%% Filter Data

Fe=1/(data(2,1)-data(1,1)); %Sampling frequency
Fc=50; % Cut-off frequency
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
    end
end

for i = 2:size(data,2)
    split_label = strsplit(labels{i},'_');
    comp = split_label{end};
    frc_plate_num = str2double(split_label{end-1});

    start = -1;
    stop = -1;    
    for t = 1:size(data,1)
        if(data(t,vert_frc_ind(frc_plate_num)) > vert_frc_threshold)
            proc_data(t,i) = data(t,i);
            
            if(start == -1)
                start = t;                
            end
        else
            proc_data(t,i) = 0;
            
            if(start ~= -1 && stop == -1)
                stop = t-1;
            end
        end     
    end 
    if(start > -1)
        pad_size =Fe/2;
        pad_data = padarray(proc_data(start:stop,i),pad_size,'replicate','both');
        %plot(pad_data)
        filt_pad_data = filtfilt(B,A,pad_data);
    
        filt_data(start:stop,i) = filt_pad_data(pad_size+1:end-pad_size);
    end
end

for i = 2:size(data,2)
    split_label = strsplit(labels{i},'_');
    comp = split_label{end};

%     filt_data(:,i) = filtfilt(B,A,proc_data(:,i));
    frc_plate_num = split_label{end-1};
    
    if(str2double(frc_plate_num) < 4)
        figure('name',labels{i});hold on;
        plot(data(:,1),data(:,i),'r');
        plot(filt_data(:,1),filt_data(:,i),'b');
    end
    
end

write_opensim_mot(out_file,data,labels,header)