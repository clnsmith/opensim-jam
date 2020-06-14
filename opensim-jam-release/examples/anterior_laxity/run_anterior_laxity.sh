#!/bin/sh

#============================
# Run Anterior Laxity Example
#============================

BIN=$PWD/../../bin/centos7

export LD_LIBRARY_PATH=$BIN:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

$BIN/forsim $BIN/libjam_plugin.so ./inputs/forsim_settings.xml