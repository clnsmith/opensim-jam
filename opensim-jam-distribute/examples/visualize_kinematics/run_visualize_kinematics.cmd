set BIN=..\..\..\bin
set OPENSIM=..\..\opensim

rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\inputs\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

pause