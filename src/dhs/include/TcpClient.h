#ifndef __TCPCLIENT_H__
#define __TCPCLIENT_H__

#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
	#include <winsock2.h>
	#include <io.h>
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <unistd.h>
	#include <netdb.h>
#endif

#include <cv.h>
#include <highgui.h>

class TcpClient {
public:
	int sockfd;
protected:
     struct sockaddr_in serv_addr;
public:
	TcpClient();
	virtual ~TcpClient();
	bool connect(const char* host, int port);
	void close();
	bool send(const void* buffer, int len);
	bool send(const char* buffer);
    bool send(cv::Mat &image);
    int receive(char *buffer, int len);

	bool connected;
};

#endif
