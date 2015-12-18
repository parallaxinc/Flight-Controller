;
; Parallax GroundStation install script
;

[Setup]
AppName=Parallax GroundStation
AppVersion=1.0.2.0
AppSupportURL=https://www.parallax.com/support
AppSupportPhone=1-916-624-8333

DefaultDirName={pf}\Parallax Inc\GroundStation
DefaultGroupName=Parallax Inc\GroundStation
UninstallDisplayIcon={app}\Parallax-GroundStation.exe
Compression=lzma2
SolidCompression=yes
OutputDir=.
OutputBaseFilename=Install-Parallax-GroundStation-V102

WizardImageFile=InstallerLogo.bmp
WizardImageStretch=yes

[Tasks]
Name: desktopicon; Description: "Create a desktop icon"; GroupDescription: "Additional icons:";

[Files]
Source: "Elev8-Groundstation\bin\release\Parallax-GroundStation.exe"; DestDir: "{app}";
Source: "Elev8-Groundstation\bin\release\Parallax-Groundstation.pdb"; DestDir: "{app}"
Source: "Elev8-Groundstation\bin\release\FTD2XX_NET.dll"; DestDir: "{app}"

[Icons]
Name: "{group}\GroundStation"; Filename: "{app}\Parallax-GroundStation.exe"
Name: "{group}\Uninstall GroundStation"; Filename: "{uninstallexe}"
Name: "{commondesktop}\Parallax GroundStation"; Filename: "{app}\Parallax-GroundStation.exe"; Tasks: desktopicon
