;
; Parallax GroundStation source install script
;

[Setup]
AppName=Parallax GroundStation Sources
AppVersion=1.0
AppSupportURL=https://www.parallax.com/support
AppSupportPhone=1-916-624-8333

DefaultDirName={userdocs}\Parallax Inc\Elev8-FC
Compression=lzma2
SolidCompression=yes
OutputDir=.
OutputBaseFilename=Install-Parallax-GroundStation-Sources

WizardImageFile=InstallerLogo.bmp
WizardImageStretch=yes

[Files]
Source: "Elev8-GroundStation\*.cs"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.resx"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.png"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.dll"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.xml"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.config"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.csproj"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\*.sln"; DestDir: "{app}\Elev8-GroundStation"
Source: "Elev8-GroundStation\Controls\*.*"; DestDir: "{app}\Elev8-GroundStation\Controls"
Source: "Elev8-GroundStation\GraphLib\*.*"; DestDir: "{app}\Elev8-GroundStation\GraphLib"
Source: "Elev8-GroundStation\Instruments\*.*"; DestDir: "{app}\Elev8-GroundStation\Instruments"
Source: "Elev8-GroundStation\Properties\*.*"; DestDir: "{app}\Elev8-GroundStation\Properties"
Source: "Elev8-GroundStation\Resources\*.*"; DestDir: "{app}\Elev8-GroundStation\Resources"

[Icons]
Name: "{group}\Uninstall GroundStation Sources"; Filename: "{uninstallexe}"
