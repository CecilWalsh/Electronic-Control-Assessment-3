#include "SerialExample.hpp"


const std::string keys{
	"{help h usage ? |      | print this message   }"
	"{com-port p     |      | serial com port      }"
};

int main(int argc, char** argv) {
	
	io::Gimbal gimbal("COM3");

}