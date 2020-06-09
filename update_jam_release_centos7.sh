#!/bin/sh

#Copy OpenSim Libraries and Executables
tar xzf ~/compile_opensim/opensim-core-install.tar.gz -C ~/compile_opensim/
#cp ~/compile_opensim/install/bin/* ./opensim-jam-release/opensim/centos7/
cp -r ~/compile_opensim/install/* ./opensim-jam-release/opensim/centos7/

chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKcommon.so
chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKcommon.so.3.7
chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKmath.so
chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKmath.so.3.7
chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKsimbody.so
chmod +x ./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64/libSimTKsimbody.so.3.7

#Copy OpenSim JAM Libraries and Executables
tar xzf ~/compile_opensim_jam/opensim-jam-install.tar.gz

cp ~/compile_opensim_jam/install/plugin/* ./opensim-jam-release/bin/centos7/
cp ~/compile_opensim_jam/install/cmd_tools/* ./opensim-jam-release/bin/centos7/
cp ~/compile_opensim_jam/install/jam_tools/* ./opensim-jam-release/bin/centos7/

