CFLAGS=-Iinclude -I../src -O3 -Wall -lboost_system -lpthread -ldcam -std=c++11


all:
	mkdir -p bin
	g++ src/x3server.cpp  ${CFLAGS} -o bin/x3server 
	g++ src/x3client.cpp  ${CFLAGS} -o bin/x3client 

clean:
	rm bin/*

install:
	mkdir -p /usr/local/bin
	cp scripts/start_x3server /usr/local/bin/
	cp bin/x3server /usr/local/bin/
	cp scripts/x3server.conf /etc/init/
	rm -f /etc/init.d/x3server
	ln -s /etc/init/x3server.conf /etc/init.d/x3server

uninstall:
	rm -f /etc/init.d/x3server
	rm -f /etc/init/x3server.conf
	rm -f /usr/local/bin/x3server
	rm -f /usr/local/bin/start_x3server