#!/bin/sh

#====================
# Run Walking Example
#====================

BIN=$PWD/../../bin/centos7

export LD_LIBRARY_PATH=$BIN:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

$BIN/comak $BIN/libjam_plugin.so ./inputs/comak_settings.xml


#COMAK Inverse Kinematics
$BIN/comak-inverse-kinematics $BIN/libjam_plugin.so ./inputs/comak_inverse_kinematics_settings.xml
#move out.log results\comak-inverse-kinematics_out.log
#move err.log results\comak-inverse-kinematics_err.log

#COMAK
$BIN/comak $BIN/libjam_plugin.so ./inputs/comak_settings.xml
#move out.log results\comak_out.log
#move err.log results\comak_err.log

#Joint Mechanics Analysis
$BIN/joint-mechanics $BIN/libjam_plugin.so ./inputs/joint_mechanics_settings.xml 
#move out.log results\joint_mechanics_out.log
#move err.log results\joint_mechanics_err.log

#Inverse Dynamics
$BIN/opensim-cmd  -L $BIN/libjam_plugin.so run-tool ./inputs/inverse_dynamics_settings.xml 
#move out.log results\inverse_dynamics_out.log
#move err.log results\inverse_dynamics_err.log