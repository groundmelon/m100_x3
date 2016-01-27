CFLAGS=-Iinclude -I../src -O3 -Wall -lboost_system -lpthread -ldcam -std=c++11

all:
	mkdir -p bin
	g++ src/x3server.cpp  ${CFLAGS} -o bin/x3server 
	g++ src/x3client.cpp  ${CFLAGS} -o bin/x3client 
clean:
	rm bin/*