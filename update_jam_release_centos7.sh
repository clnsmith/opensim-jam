#!/bin/sh

#Copy OpenSim Libraries and Executables
BIN=./opensim-jam-release/bin/centos7/
SIMBODY=./opensim-jam-release/opensim/centos7/sdk/Simbody/lib64
OPENSIM_EXE=./opensim-jam-release/opensim/centos7/bin
OPENSIM_LIB=./opensim-jam-release/opensim/centos7/sdk/lib

tar xzf ~/compile_opensim/opensim-core-install.tar.gz -C ~/compile_opensim/
cp -r ~/compile_opensim/install/* ./opensim-jam-release/opensim/centos7/

chmod +x $SIMBODY/libSimTKcommon.so
chmod +x $SIMBODY/libSimTKcommon.so.3.7
chmod +x $SIMBODY/libSimTKmath.so
chmod +x $SIMBODY/libSimTKmath.so.3.7
chmod +x $SIMBODY/libSimTKsimbody.so
chmod +x $SIMBODY/libSimTKsimbody.so.3.7

cp $SIMBODY/* $BIN/
cp $OPENSIM_EXE/* $BIN/
cp $OPENSIM_LIB/* $BIN/

#Copy OpenSim JAM Libraries and Executables
tar xzf ~/compile_opensim_jam/opensim-jam-install.tar.gz

cp ~/compile_opensim_jam/install/plugin/* ./opensim-jam-release/bin/centos7/
cp ~/compile_opensim_jam/install/cmd_tools/* ./opensim-jam-release/bin/centos7/
cp ~/compile_opensim_jam/install/jam_tools/* ./opensim-jam-release/bin/centos7/

