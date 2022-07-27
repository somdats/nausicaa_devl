

#include <thread>
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\server.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\nausicaa_api_server.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\deserialize.h"





void acceptingC() {
	while(true)
		serverComm.accepting_connections();
}

void incomingC(std::string & message) {
	int i = 0;
	while (true)
	{
		if (serverComm.incoming_message(message)) {
			i++;
			serverComm.send(std::to_string(i));
		}
	}
}

void acceptingS() {
	while (true)
		serverStream.accepting_connections();
}

void incomingS(std::string& message) {
	int i = 0;
	while (true)
	{
		if (serverStream.incoming_message(message)) {
			i++;
			serverStream.send(std::to_string(i));
		}
	}
}

int main(int argc, char **argv) {
	std::string messageC, messageS;
	 
	//call_API_function(std::string("pippo@7@2.03f@"));



	serverComm.start_server(81);
	std::thread t0_comm = std::thread(&acceptingC);
	std::thread t1_comm = std::thread(&incomingC,messageC);

	serverStream.start_server(82);
	std::thread t0_strem = std::thread(&acceptingS);
	std::thread t1_strem = std::thread(&incomingS, messageS);

	while (true);
	//accepting_connections();
//	incoming_message(message);
	printf("message %s", messageC.c_str());
}
