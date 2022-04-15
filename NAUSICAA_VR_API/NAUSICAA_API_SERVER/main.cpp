
#include "header/nausicaa_api_server.h"
#include "header/server.h"

int main() {
	std::string message;

//	start_server();
	start_server_stream();

	while (true) {
		if(incoming_message(message))
			call_API_function(message);
	}
}