%% Convert TRC Files

infile = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\prep\overground_17';
outfile = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\overground_17';

% infile = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\prep\static1';
% outfile = 'C:\Users\csmith\github\opensim-jam\source\models\lenhart2015\motion_data\static1';

[org_mrkdata,time,f,n,nmrk,mrk_names,file,inpath]=load_trc(infile,'');
%[trcData,markerLabels,headerStruc,trcData3D] = readTRCFile(infile);

%% rotate coordinate system
%org_x=x
%org_y=z
%org_z=y
[nRow,nCol] = size(org_mrkdata);
mrkdata = zeros(nRow,nCol);

%% Filter Marker Positions
Fe=1/(time(2,1)-time(1,1)); %Sampling frequency
Fc=6; % Cut-off frequency
N=4; % Filter Order
[B, A] = butter(N,Fc*2/Fe, 'low');


for i = 1:nmrk
    ind = (i-1)*3;
    x_data = filtfilt(B,A,org_mrkdata(:,ind+1));
    y_data = filtfilt(B,A,org_mrkdata(:,ind+2));
    z_data = filtfilt(B,A,org_mrkdata(:,ind+3));
    
    mrkdata(:,ind+1) = x_data;
    mrkdata(:,ind+2) = z_data;
    mrkdata(:,ind+3) = -y_data;
end

%% Remove HJC


nmd=1;
mnm=1;
for i = 1:nmrk
    if(~contains(mrk_names{i},'HJC'))
        ind = (i-1)*3;
        nmd_ind = nmd-1;
        final_mrkdata(:,nmd) = mrkdata(:,ind+1);
        final_mrkdata(:,nmd+1) = mrkdata(:,ind+2);
        final_mrkdata(:,nmd+2) = mrkdata(:,ind+3);
        
        nmd=nmd+3;
        final_mrk_names{mnm} = mrk_names{i};
        mnm=mnm+1;
    end
end

writeTRCFile(time,final_mrkdata,final_mrk_names','',outfile)