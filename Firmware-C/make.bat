
SET LIBS=F:\Users\Jason\Documents\SimpleIDE\Learn\Simple Libraries

REM propeller-elf-gcc.exe -v GCC 4.6.1 (propellergcc_v1_0_0_2408)

propeller-elf-gcc.exe -I . -L . -I "%LIBS%/Utility/libsimpletools" -L "%LIBS%/Utility/libsimpletools/cmm/" -I "%LIBS%/TextDevices/libsimpletext" -L "%LIBS%/TextDevices/libsimpletext/cmm/" -I "%LIBS%/Protocol/libsimplei2c" -L "%LIBS%/Protocol/libsimplei2c/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -c Servo32_HighRes.c -o cmm/Servo32_HighRes.o

propeller-elf-gcc.exe -I . -L . -I "%LIBS%/Utility/libsimpletools" -L "%LIBS%/Utility/libsimpletools/cmm/" -I "%LIBS%/TextDevices/libsimpletext" -L "%LIBS%/TextDevices/libsimpletext/cmm/" -I "%LIBS%/Protocol/libsimplei2c" -L "%LIBS%/Protocol/libsimplei2c/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -c Servo32_HighRes_driver.sx -o cmm/Servo32_HighRes_driver.o


propeller-elf-gcc.exe -I . -L . -I "%LIBS%/Utility/libsimpletools" -L "%LIBS%/Utility/libsimpletools/cmm/" -I "%LIBS%/TextDevices/libsimpletext" -L "%LIBS%/TextDevices/libsimpletext/cmm/" -I "%LIBS%/Protocol/libsimplei2c" -L "%LIBS%/Protocol/libsimplei2c/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -c RC_Receiver.c -o cmm/RC_Receiver.o

propeller-elf-gcc.exe -I . -L . -I "%LIBS%/Utility/libsimpletools" -L "%LIBS%/Utility/libsimpletools/cmm/" -I "%LIBS%/TextDevices/libsimpletext" -L "%LIBS%/TextDevices/libsimpletext/cmm/" -I "%LIBS%/Protocol/libsimplei2c" -L "%LIBS%/Protocol/libsimplei2c/cmm/" -Os -mcmm -m32bit-doubles -fno-exceptions -c RC_Receiver_driver.sx -o cmm/RC_Receiver_driver.o

propeller-elf-gcc.exe -I . -L . -I "%LIBS%/Utility/libsimpletools" -L "%LIBS%/Utility/libsimpletools/cmm/" -I "%LIBS%/TextDevices/libsimpletext" -L "%LIBS%/TextDevices/libsimpletext/cmm/" -I "%LIBS%/Protocol/libsimplei2c" -L "%LIBS%/Protocol/libsimplei2c/cmm/" -o cmm/Elev8-Main.elf -Os -mcmm -m32bit-doubles -fno-exceptions cmm/RC_Receiver.o cmm/RC_Receiver_driver.o cmm/Servo32_HighRes.o cmm/Servo32_HighRes_driver.o -ffunction-sections -fdata-sections -Wl,--gc-sections Elev8-Main.c -lsimpletools -lsimpletext -lsimplei2c -lsimpletools -lsimpletext -lsimpletools
