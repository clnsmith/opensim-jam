#!/bin/sh

#============================
# Run Anterior Laxity Example
# ===========================

BIN=$PWD/../../bin/centos7
OPENSIM=$PWD/../../opensim/centos7/sdk/lib
SIMBODY=$PWD/../../opensim/centos7/sdk/Simbody/lib64

export LD_LIBRARY_PATH=$BIN:$OPENSIM:$SIMBODY:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

$BIN/forsim $BIN/libjam_plugin.so ./inputs/forsim_settings.xml