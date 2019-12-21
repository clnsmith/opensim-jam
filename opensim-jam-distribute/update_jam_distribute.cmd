rem Copy C++ executables and libraries
copy ..\..\install\plugin\jam_plugin.dll bin\
copy ..\..\install\cmd_tools\comak.exe bin\
copy ..\..\install\cmd_tools\comak-inverse-kinematics.exe bin\
copy ..\..\install\cmd_tools\forsim.exe bin\
copy ..\..\install\cmd_tools\joint-mechanics.exe bin\

rem Copy Models
xcopy /s /y ..\models  .\models 
pause