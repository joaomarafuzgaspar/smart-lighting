#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include "serial_interface.h"
#include "server.h"
#include "session.h"

int main(int argc, char* argv[])
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " <port>\n";
            return 1;
        }

        boost::asio::io_context io_context;
        SerialInterface iterface {io_context, "/dev/ttyS10", 115200};
        SCDTRServer s(io_context, std::atoi(argv[1]));

        io_context.run();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}