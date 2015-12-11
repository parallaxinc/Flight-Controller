;
; Parallax Elev8 FC firmware source install script
;

[Setup]
AppName=Parallax Elev8-FC Firmware Source
AppVersion=1.0
AppSupportURL=https://www.parallax.com/support
AppSupportPhone=1-916-624-8333

DefaultDirName={userdocs}\Parallax Inc\Elev8-FC
Compression=lzma2
SolidCompression=yes
OutputDir=.
OutputBaseFilename=Install-Parallax-Elev8FC-Sources

WizardImageFile=InstallerLogo.bmp
WizardImageStretch=yes

[Files]
Source: "Firmware-C\*.c*"; DestDir: "{app}\Firmware"
Source: "Firmware-C\*.h"; DestDir: "{app}\Firmware"
Source: "Firmware-C\*.side"; DestDir: "{app}\Firmware"
Source: "Firmware-C\*.spin"; DestDir: "{app}\Firmware"
Source: "Firmware-C\readme.txt"; DestDir: "{app}\Firmware"
Source: "Firmware-C\LICENSE"; DestDir: "{app}\Firmware"

[Icons]
Name: "{group}\Uninstall Elev8-FC Sources"; Filename: "{uninstallexe}"
