#include "stdafx.h"
#include "WindowsSocket.h"
#include <iostream>
#include <string>
using namespace std;

#define WM_MICHAEL WM_USER+100
extern BOOLEAN OnlineOn;
socketThreadInfo Info;
void WindowsSocket::ListenTCPClient(void)
{
	/* Step 1: startup winsocket - this is for Windows only */
	/* in pair with WSACleanup()    */

	//ͨ���������ƣ���ȡ�������̴��ھ��
	CWnd *pWnd=CWnd::FindWindow(NULL,_T("Avoidance"));
	
	CString str;
	str.Format(_T("******** TCP Server ********\r\n\r\n"));
	pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));

	//char s2[50]="like";

	//str.Format(_T("******** TCP Server ********\r\n\r\n %S"),s2);
	//pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));

	if(WSAStartup(MAKEWORD(2,2), &wsa_data) != 0)
	{
		puts("WSAStartup failed!");
		exit(1);
	}

	/* Step 2: Create socket and check it is successful */
	/* in pair with closesocket()   */
	TCPListen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(TCPListen == SOCKET_ERROR)
	{
		str.Format(_T("Failed to create scoket(): %d\n"),WSAGetLastError());
		pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));
		exit(1);
	}

	/* Step 3.1: Setup Parameters for local TCP Server*/
	local.sin_family = AF_INET;
	local.sin_addr.s_addr = inet_addr(MY_SERVER_IP_ADDRESS); /* defined in tcpserverclient.h */
	local.sin_port = htons(MY_SERVER_PORT_NUMBER);           /* defined in tcpserverclient.h */

	/* Step 3.2: Bind to the local TCP Server */
	if(bind(TCPListen, (struct sockaddr *)&local, sizeof(local)) == SOCKET_ERROR)
	{
		str.Format(_T("Failed to bind(): %d\n"),WSAGetLastError());
		pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));
		exit(1);
	}

	/* Step 4: Listen */
	listen(TCPListen, 8);

	/* Step 5: Accept in Loops*/
	while(OnlineOn)
	{
		/* Step 5.1: Accept */
		ipAddrSize = sizeof(client);
		TCPClient = accept(TCPListen, (struct sockaddr *)&client, &ipAddrSize);
		if(TCPClient == INVALID_SOCKET)
		{
			str.Format(_T("Failed to accept(): %d\n"),WSAGetLastError());
			pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));
			exit(1);
		}
		CString str1;
		str1.Format(_T("Client Accepted: %S:%d\n"),inet_ntoa(client.sin_addr),ntohs(client.sin_port));
		pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str1));

		/* Step 5.2: Send and Receive Data in loops */
		iterationStep = 1;
		while(OnlineOn)
		{
			recvStatus = recv(TCPClient, recvBuffer,128,0);
			if(recvStatus == 0)
				break;
			else if(recvStatus == SOCKET_ERROR)
			{
				str.Format(_T("Failed in recv(): %d\n"),WSAGetLastError());
				pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));
				break;
			}
			recvBuffer[recvStatus] = 0x00; /* '\0' */

			str.Format(_T("Step = %d>>%S\n"),iterationStep,recvBuffer);
			pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));
			iterationStep++;
		}
	}
	/* Step 6: Close socket, in pair with socket() */ 
	closesocket(TCPListen);
	/* Step 7: Clean up winsocket - this is for Windows only! */
	/* in pair with WSAStartup() */
	WSACleanup();
}

void WindowsSocket::SendToTCPClient(void)
{
	Info.pClass=this;
	hThread = CreateThread(NULL,
		0,
		(LPTHREAD_START_ROUTINE)SocketThreadSend,
		&Info,
		0,
		&ThreadID);
}

void WindowsSocket::ShutDownBoth(void)
{
	shutdown(TCPListen,SD_BOTH);
	/* Step 6: Close socket, in pair with socket() */ 
	closesocket(TCPListen);
	/* Step 7: Clean up winsocket - this is for Windows only! */
	/* in pair with WSAStartup() */
	WSACleanup();
}

UINT SocketThreadSend(LPVOID lpParam)
{	
	socketThreadInfo* pInfo = (socketThreadInfo*)lpParam;

	//windowsSocket.ListenTCPClient();
	CWnd *pWnd=CWnd::FindWindow(NULL,_T("Avoidance"));
	CString str;
	str.Format(_T("�����߳̿���\r\n\r\n"));
	pWnd->SendMessage(WM_MICHAEL,0,LPARAM(&str));

	return 0;
}