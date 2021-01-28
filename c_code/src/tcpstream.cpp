/*
   TCPStream.h

   TCPStream class definition. TCPStream provides methods to trasnfer
   data between peers over a TCP/IP connection.

   ------------------------------------------

   Copyright (c) 2013 Vic Hargrave

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <iostream>
#include <arpa/inet.h>
#include "tcpstream.h"


TCPStream::TCPStream(int sd, struct sockaddr_in* address) : m_sd(sd) {
    char ip[50];
    inet_ntop(PF_INET, (struct in_addr*)&(address->sin_addr.s_addr), ip, sizeof(ip)-1);
    m_peerIP = ip;
    m_peerPort = ntohs(address->sin_port);
}

TCPStream::~TCPStream()
{
    close(m_sd);
}

ssize_t TCPStream::send(const char* buffer, size_t len) 
{
    return write(m_sd, buffer, len);
}

ssize_t TCPStream::receive(char* buffer, size_t len, int timeout) 
{
    if (timeout <= 0) return read(m_sd, buffer, len);

    if (waitForReadEvent(timeout) == true) {
      return read(m_sd, buffer, len);
    } else {
      return read(m_sd, buffer, len);
    }
    return connectionTimedOut;

}


string TCPStream::getPeerIP() 
{
    return m_peerIP;
}

int TCPStream::getPeerPort() 
{
    return m_peerPort;
}

bool TCPStream::waitForReadEvent(int timeout)
{
    fd_set sdset;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = timeout;
    FD_ZERO(&sdset);
    FD_SET(m_sd, &sdset);
	int retval=0;
	retval = select(m_sd+1, &sdset, NULL, NULL, &tv);
	if( retval<0 ){
	  std::cout<<"waitForReadEvent: negative return value of select."<<std::endl;
	} else if( retval>0 ){
	  return true;	
	} else {
	  std::cout<<"Timeout in waitForReadEvent."<<std::endl;	
	}
	
/*    // if (select(m_sd+1, &sdset, NULL, NULL, &tv) > 0) {
    if (select(m_sd+1, &sdset, NULL, NULL, &tv) > 0 && FD_ISSET(m_sd, &sdset)) {
      return true;
    } else if (select(m_sd+1, &sdset, NULL, NULL, &tv) == 0) {
      std::cout<<"Timeout: waiting for the DAQ to free some space."<<std::endl;
      return false;
    }
	*/
    return false;
}
