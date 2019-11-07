set BIN=..\..\..\bin
set OPENSIM=..\..\opensim

rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

REM %BIN%\forsim %BIN%\jam_plugin.dll .\settings\forsim_settings.xml
REM move out.log results\forsim_out.log
REM move err.log results\forsim_err.log

%BIN%\joint-mechanics %BIN%\jam_plugin.dll .\settings\joint_mechanics_settings.xml 
move out.log results\joint_mechanics_out.log
move err.log results\joint_mechanics_err.log

pause