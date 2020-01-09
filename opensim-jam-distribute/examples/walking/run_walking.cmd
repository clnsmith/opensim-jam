set BIN=..\..\..\bin
set BIN=C:\Users\csmith\github\opensim-jam\source\opensim-jam-distribute\bin
set OPENSIM=..\..\opensim


rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

rem Scale Model
REM %OPENSIM%\opensim-cmd  -L %BIN%\jam_plugin.dll run-tool .\inputs\scale_settings.xml 
REM move out.log results\scale_out.log
REM move err.log results\scale_err.log

rem Inverse Kinematics
REM %BIN%\comak-inverse-kinematics %BIN%\jam_plugin.dll .\inputs\comak_inverse_kinematics_settings.xml
REM move out.log results\comak-inverse-kinematics_out.log
REM move err.log results\comak-inverse-kinematics_err.log

rem %BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\ik_constraint_sim_joint_mechanics_settings.xml 
rem move out.log results\joint_mechanics_out.log
rem move err.log results\joint_mechanics_err.log

rem COMAK
REM %BIN%\comak %BIN%\jam_plugin.dll .\inputs\comak_settings.xml
REM move out.log results\comak_out.log
REM move err.log results\comak_err.log

REM Joint Mechanics Analysis
%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

rem Inverse Dynamics
REM %OPENSIM%\opensim-cmd  -L %BIN%\jam_plugin.dll run-tool .\inputs\inverse_dynamics_settings.xml 
REM move out.log results\inverse_dynamics_out.log
REM move err.log results\inverse_dynamics_err.log

pause