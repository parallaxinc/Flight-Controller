set OLDPATH=%path%
set path=F:\qt\5.5\mingw492_32\bin;F:\Qt\Tools\mingw492_32\bin;%PATH%
F:
cd \github\flight-controller
rd /s /q GroundStation-Release

mkdir GroundStation-Release
cd GroundStation-Release

copy F:\QtApps\build-GroundStation-Desktop_Qt_5_5_1_MinGW_32bit-Release\release\groundstation.exe .
windeployqt .
cd ..

"C:\Program Files (x86)\Inno Setup 5\ISCC.exe" InstallGroundStation-Qt.iss
set PATH=%oldpath%
