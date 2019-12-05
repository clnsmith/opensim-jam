set BIN=..\..\..\bin
set BIN=C:\Users\csmith\github\opensim-jam\source\opensim-jam-distribute\bin
set OPENSIM=..\..\opensim


rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

REM Perform Forward Simulation
%BIN%\forsim %BIN%\jam_plugin.dll .\inputs\forsim_settings.xml
move out.log results\forsim_out.log
move err.log results\forsim_err.log

REM Analze results using joint-mechanics 
%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

pause