set BIN=..\..\..\bin
set BIN=C:\Users\csmith\github\opensim-jam\source\opensim-jam-distribute\bin
set OPENSIM=..\..\opensim


rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

%BIN%\forsim %BIN%\jam_plugin.dll .\settings\forsim_settings.xml
move out.log results\forsim_out.log
move err.log results\forsim_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\settings\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

pause