// #include "src/drone.h"
#include <thread>
#include <Gobbledegook.h>
#include "src/ble/bleserver.h"

int main(int argc, char **ppArgv) {
    // init();
    // runEventLoop();

    blemain(argc, ppArgv);

    // do something to wait
    while (ggkGetServerRunState() < EStopping)
	{
		std::this_thread::sleep_for(std::chrono::seconds(15));
	}

    return 0;
}
