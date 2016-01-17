#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<winsock2.h>
#include <windows.h>    /* Note: winsock2.h has included windows.h */

#pragma comment(lib,"Ws2_32.lib")
/* IP address of TCP server to be connected to  */
#define MY_SERVER_IP_ADDRESS     "172.26.213.6"   
// #define MY_SERVER_IP_ADDRESS     "192.168.1.12"   
// #define IP_ADDRESS     "10.171.128.251"   
/* #define IP_ADDRESS     "10.0.1.167" */

/* port no. on TCP server to be connected to */
#define MY_SERVER_PORT_NUMBER    3247        

/* maximum number of iterations of sending data */
#define MAXCOUNT 15

class WindowsSocket  
{
public:
	//data
	WSADATA  wsa_data;                /* type defined in winsock2.h */
	SOCKET   TCPListen, TCPClient;    /* type defined in winsock2.h */
	struct sockaddr_in local,client;  /* struct defined in winsock2.h */
	int      ipAddrSize, recvStatus, sendStatus, iterationStep;             
	char     recvBuffer[128];
	double   time_old, time_new, time_interval;
	HANDLE hThread;
	DWORD ThreadID;
public:
	//functions
	void ListenTCPClient(void);
	void SendCStringToTCPClient(CString);
	void SendIntArrayToTCPClient(int*);
	void ShutDownBoth(void);
};

struct socketThreadInfo
{
	int* pRangedata;
	WindowsSocket* pClass;
	CString pSendMessageStr;
	int* pSendInt;
};

UINT SocketThreadSend(LPVOID lpParam);
UINT SocketThreadSendArray(LPVOID lpParam);