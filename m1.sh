arm-linux-gnueabihf-g++ -Wall  -c imufilters.c

arm-linux-gnueabihf-g++ -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -O3 -c testgy.c 

arm-linux-gnueabihf-g++ -Wall -c -O3 imuLib.c 

#arm-linux-gnueabihf-g++ -Wall -c -O3 clock.c 


arm-linux-gnueabihf-g++ -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math -O3 -Wall -fsingle-precision-constant imufilters.o imuLib.o testgy.o -o Q

