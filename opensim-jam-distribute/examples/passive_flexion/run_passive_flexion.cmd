set BIN=%CD%\..\..\bin
set OPENSIM=%CD%\..\..\opensim

rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

REM Run Healthy Simulation
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings.xml
move out.log results\forsim_out.log
move err.log results\forsim_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

REM Run alta Simulation
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings_alta.xml
move out.log results\forsim_alta_out.log
move err.log results\forsim_alta_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings_alta.xml 
move out.log results\joint_mechanics_alta_out.log
move err.log results\joint_mechanics_alta_err.log

REM Run alta PTA normal Simulation
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings_alta_PTA_normal.xml
move out.log results\forsim_alta_PTA_normal_out.log
move err.log results\forsim_alta_PTA_normal_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings_alta_PTA_normal.xml 
move out.log results\joint_alta_PTA_normal_mechanics_out.log
move err.log results\joint_alta_PTA_normal_mechanics_err.log

REM Run alta PTA baja Simulation
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings_alta_PTA_baja.xml
move out.log results\forsim_alta_PTA_baja_out.log
move err.log results\forsim_alta_PTA_baja_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings_alta_PTA_baja.xml 
move out.log results\joint_mechanics_alta_PTA_baja_out.log
move err.log results\joint_mechanics_alta_PTA_baja_err.log

pause