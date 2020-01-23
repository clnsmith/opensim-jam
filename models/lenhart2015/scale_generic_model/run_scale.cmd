set BIN=%CD%\..\..\..\opensim-jam-release\bin
set OPENSIM=%CD%\..\..\..\opensim-jam-release\opensim

rem Let Windows know where the plugin and opensim libraries are
set PATH=%BIN%;%OPENSIM%;%PATH%

rem Scale Model
%OPENSIM%\opensim-cmd  -L %BIN%\jam_plugin.dll run-tool .\scale_settings.xml 

pause