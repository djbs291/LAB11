

PROG=ball

INC_BULLET=-I/usr/include/bullet
INC_OSG=-I/usr/include/osg
LD_OSG= -losg -losgViewer -losgSim -losgDB -losgGA -losgShadow -losgText
LD_BULLET=-l BulletDynamics -l BulletCollision -l LinearMath
BASE_DIR=http://ave.dee.isep.ipp.pt/~jml/intmu/lab11

CFLAGS=-std=c++11 -Wall -O2 -Wno-uninitialized


all: ${PROG}

${PROG}: ${PROG}.o btosg/btosg.o
	g++ -O2 -o $@ $^ ${LD_BULLET} ${LD_OSG} -lm
	
${PROG}.o: ${PROG}.cpp btosg/btosg.h 
	g++ ${CFLAGS} -c ${INC_BULLET} ${INC_OSG} $<

#-DBTOSG_SHADOW $<

	
btosg:
	git clone https://github.com/miguelleitao/btosg.git
	make -C btosg

getall: 
	wget -q ${BASE_DIR}/ball.cpp
	wget -q ${BASE_DIR}/ball.png
	wget -q ${BASE_DIR}/pino.mtl
	wget -q ${BASE_DIR}/pino.obj
	wget -q ${BASE_DIR}/beachball.png
	wget -q ${BASE_DIR}/bowling_pin.png

clean:
	rm -f *.o ${PROG}

dist: clean
	rm -rf btosg

