

#include <thread>


#include "..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\server.h"
#include "..\..\NAUSICAA_VR_API\NAUSICAA_API_SERVER\header\deserialize.h"

void accepting() {
	while(true)
		accepting_connections();
}

void incoming(std::string & message) {
	int i = 0;
	while (true)
	{
		if (incoming_message(message)) {
			i++;
			send(std::to_string(i));
		}
	}
}

int main(int argc, char **argv) {
	std::string message;
	 
	//call_API_function(std::string("pippo@7@2.03f@"));



	start_server();
	std::thread t0 = std::thread(&accepting);
	std::thread t1 = std::thread(&incoming,message);

	while (true);
	//accepting_connections();
//	incoming_message(message);
	printf("message %s", message.c_str());
}
