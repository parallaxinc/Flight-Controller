;
; Parallax GroundStation install script
;

[Setup]
AppName=Parallax GroundStation
AppVersion=1.1.1
AppSupportURL=https://www.parallax.com/support
AppSupportPhone=1-916-624-8333

DefaultDirName={pf}\Parallax Inc\GroundStation
DefaultGroupName=Parallax Inc\GroundStation
UninstallDisplayIcon={app}\GroundStation.exe
Compression=lzma2
SolidCompression=yes
OutputDir=.
OutputBaseFilename=80204-Parallax-GroundStation-v1.1.1

WizardImageFile=InstallerLogo.bmp
WizardImageStretch=yes

[Tasks]
Name: desktopicon; Description: "Create a desktop icon"; GroupDescription: "Additional icons:";

[Files]
Source: "Groundstation-Release\*.*"; DestDir: "{app}"; Flags: recursesubdirs
Source: "GroundStation-Qt\*.*"; DestDir: "{userdocs}\ParallaxInc\Elev8-Sources\GroundStation-Qt"; Flags: recursesubdirs; Excludes: "GroundStation.pro.user,.gitignore"
Source: "Firmware-C\*.*"; DestDir: "{userdocs}\ParallaxInc\Elev8-Sources\Firmware-C"; Excludes: ".gitignore"

[Icons]
Name: "{group}\Parallax GroundStation"; Filename: "{app}\GroundStation.exe"
Name: "{group}\Uninstall GroundStation"; Filename: "{uninstallexe}"
Name: "{commondesktop}\Parallax GroundStation"; Filename: "{app}\GroundStation.exe"; Tasks: desktopicon
