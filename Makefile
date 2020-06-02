
CC = gcc
CC1 = g++
PXFLAG =  -I. -I ./c_library_v2/common

mainLib=  -l ncurses -l pthread -l prussdrv -lm

options =  -O3 -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math
options1 = -Wall -fsingle-precision-constant  -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ffast-math

objects = Qmain.o Qfilters.o Qser.o Qpxserial.o Qtime.o Qpid.o Qdiagnostic.o QimuLib.o Qpru.o Qmod.o Qaltitude.o


Q: $(objects) 
	$(CC1) $(options) $(objects) -o Q $(mainLib)

Qmain.o: Qmain.c
	$(CC) $(options) -c Qmain.c -o Qmain.o

Qfilters.o: imufilters.c
	$(CC) $(options1) -c imufilters.c -o Qfilters.o

Qaltitude.o: altitudeKF.c
	$(CC) $(options) -c altitudeKF.c -o Qaltitude.o

Qser.o: toserial.c
	$(CC) $(options) -o Qser.o -c toserial.c

Qpxserial.o: PXtoserial.c
	$(CC) $(options) $(PXFLAG) -c PXtoserial.c -o Qpxserial.o

Qtime.o: clock.c
	$(CC) $(options) -c clock.c -o Qtime.o

Qpid.o: Pid.c
	$(CC) $(options) -o Qpid.o -c Pid.c

Qdiagnostic.o: element.c
	$(CC) $(options) -c element.c -o Qdiagnostic.o

QimuLib.o : imuLib.c
	$(CC) $(options) -c imuLib.c -o QimuLib.o

Qpru.o: prulib.c
	$(CC) $(options) -o Qpru.o -c prulib.c

Qmod.o: Qmodel.c
	$(CC) $(options) -o Qmod.o -c Qmodel.c

.PHONY : clean
clean :
	rm Q $(objects)

