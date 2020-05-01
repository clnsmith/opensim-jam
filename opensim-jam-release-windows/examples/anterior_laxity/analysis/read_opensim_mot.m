function [data, labels, header] = read_opensim_mot(file)
%%=========================================================================
%READ_OPENSIM_MOT
%--------------------------------------------------------------------------
%Author(s): Colin Smith
%Date: 5/14/2018

%The kneemos_matlab toolkit is a collection of code for developing and 
%analyzing musculoskeletal simulations in SIMM and OpenSIM. The developers
%are based at the University of Wisconsin-Madison and ETH Zurich. Please
%see the README.md file for more details. It is your responsibility to
%ensure this code works correctly for your use cases. 
%
%Licensed under the Apache License, Version 2.0 (the "License"); you may
%not use this file except in compliance with the License. You may obtain a 
%copy of the License at http://www.apache.org/licenses/LICENSE-2.0.
%Unless required by applicable law or agreed to in writing, software 
%distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
%WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
%License for the specific language governing permissions and limitations 
%under the License.
%
%--------------------------------------------------------------------------
%file : str
%
%
%data : [nFrames x nLabels] matrix
%
%
%labels : {1 x nLabels} cell of strings 
%
%
%header : struct
%
%
%==========================================================================

if ~exist('file','var')
    [infile, inpath]=uigetfile('*.mot','Select input file');
    file=[inpath infile];
end
        
fid=fopen(file,'r');

if fid <0
    mot=[];labels=[];
    disp('File Not Found:\n');
    disp([file '\n']);
    return
end

disp(['Loading file...' file] );


%read the file name line
header.filename=strtrim(fgetl(fid));


% Read Header
line = fgetl(fid);
while ~strncmpi(line,'endheader',length('endheader'))

    
    if (line == -1)
        disp('ERROR: Reached EOF before "endheader"')
        return
    end
    line_space = strrep(line,'=',' ');
    split_line = strsplit(line_space);
    
    if (length(split_line)==2)
        var = split_line{1};
        value = split_line{2};

        if strcmpi(var,'version') 
            header.version = str2double(value);
        elseif strcmpi(var,'nRows') || strcmpi(var,'datarows')
            nr = str2double(value);
            header.nRows = nr;
        elseif strcmpi(var,'nColumns') || strcmpi(var,'datacolumns')
            nc = str2double(value);
            header.nColumns = nc;
        elseif strcmpi(var,'indegrees')
            header.indegrees = strtrim(value);
        end
    end
    
    line = fgetl(fid);
end


%Load Column Headers
line=fgetl(fid);

labels=cell(nc,1);

j=1;
jl=length(line);
for i=1:nc
	name=sscanf(line(j:jl),'%s',1);
    ii = findstr(line(j:jl), name);
    j=j+ii(1)+length(name);
	labels(i,1)=cellstr(name);
end

% Now load the data	
data=zeros(nr,nc);
i=0;
while ((feof(fid)==0)&(i<nr))
     i=i+1;
     line=fgetl(fid);
     data(i,:)=sscanf(line,'%f');
end
if (i<nr)
	disp(['Number of rows (',num2str(i),') is less than that specified in header (',num2str(nr),')']);
	data=data(1:i,:);
end
fclose(fid);



if (nargout>1)
    % return all data in a single array
    mot=data;
elseif (nargout==1)
    % return all information in a single structure
    mot.data=data;
    mot.hdr=labels;
end


function [t,q]=load_exp(file);
global NQ NM;
rawdata=load_motionfile(file);
[t,data]=extractcolumns(rawdata,1);
[q,data]=extractcolumns(data,NQ);


function [x,outdata]=extractcolumns(data,nc);
x=data(:,1:nc);
[m,n]=size(data);
outdata=data(:,(nc+1):n);