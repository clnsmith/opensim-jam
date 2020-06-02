REM Copy C++ executables and libraries
copy ..\install\plugin\jam_plugin.dll opensim-jam-release\bin\
copy ..\install\cmd_tools\comak.exe opensim-jam-release\bin\
copy ..\install\cmd_tools\comak-inverse-kinematics.exe opensim-jam-release\bin\
copy ..\install\cmd_tools\forsim.exe opensim-jam-release\bin\
copy ..\install\cmd_tools\joint-mechanics.exe opensim-jam-release\bin\

REM Copy Models
xcopy /I /y  .\models\lenhart2015  opensim-jam-release\models\lenhart2015 
xcopy /I /y  .\models\lenhart2015\Geometry  opensim-jam-release\models\lenhart2015\Geometry
xcopy /I /y  .\models\lenhart2015\motion_data  opensim-jam-release\models\lenhart2015\motion_data
xcopy /I /y  .\models\lenhart2015\graphics  opensim-jam-release\models\lenhart2015\graphics

xcopy /I /y  .\models\smith2019  opensim-jam-release\models\smith2019 
xcopy /I /y  .\models\smith2019\Geometry  opensim-jam-release\models\smith2019\Geometry
xcopy /I /y  .\models\smith2019\motion_data  opensim-jam-release\models\smith2019\motion_data
xcopy /I /y  .\models\smith2019\graphics  opensim-jam-release\models\smith2019\graphics

REM Copy Documentation
pause