REM ===========================
REM Run Anterior Laxity Example
REM ===========================


set BIN=%CD%\..\..\bin\win
set OPENSIM=%CD%\..\..\opensim\win

REM Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

REM Perform Forward Simulation - Healthy
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings.xml
move out.log results\forsim_out.log
move err.log results\forsim_err.log

REM Perform Joint Mechanics Analysis - Healthy
%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

REM Perform Forward Simulation - ACL Deficient 
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings_acld.xml
move out.log results\forsim_acld_out.log
move err.log results\forsim_acld_err.log

REM Perform Joint Mechanics Analysis - ACL Deficient
%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings_acld.xml 
move out.log results\joint_mechanics_acld_out.log
move err.log results\joint_mechanics_acld_err.log

pause