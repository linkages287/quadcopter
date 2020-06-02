echo "removing all obj files"
rm *.o
echo "compiling libraries" 
 
echo "compiling Q-Model MODULE"
gcc -O3 -Wall -fsingle-precision-constant -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qmod.o -c Qmodel.c 

echo "compiling PRU"
gcc -O3 -Wall -fsingle-precision-constant -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -ffast-math -o Qpru.o -c prulib.c

echo "compiling PID MODULE"
gcc -O3  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qpid.o -c Pid.c 

echo "compiling BLE serial MODULE"
gcc -O3  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qser.o -c bletoserial.c
 
echo "compiling PX4 serial MODULE"
gcc  -O3 -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math  -I. -I ./c_library_v2/common -c PXtoserial.c -o Qpxserial.o

echo "compiling timer MODULE"
gcc -O3  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qtime.o -c clock.c

echo "compiling diagnostic MODULE"
gcc -O3  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qdiagnostic.o -c element.c

echo "compiling IMU MODULE"
gcc  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o Qfilters.o -c imufilters.c

gcc -O3  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -o QimuLib.o -c imuLib.c

echo "compiling BMP180 MODULE"
gcc -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -c bmp180.c -o Qbmp180.o -lm


echo "Compiling MAIN MODULE"
gcc  -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -O3 -c Qmain.c

echo "Linking main"
g++ -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -O3 -Wall -fsingle-precision-constant Qmain.o Qser.o Qpxserial.o Qtime.o Qpid.o Qdiagnostic.o QimuLib.o Qbmp180.o Qpru.o Qmod.o Qfilters.o -l ncurses -l pthread -l prussdrv -lm -o Q

#According to [1] and [2], g++ is equivalent to gcc -xc++ -lstdc++ -shared-libgcc (the 1st is a compiler option, the 2nd two are linker options). This can be checked by running both with the -v option (it displays the backend toolchain commands being run).%
