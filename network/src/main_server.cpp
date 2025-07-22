#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include "serial_interface.h"
#include "server.h"
#include "session.h"

int main(int argc, char* argv[])
{
    try {
        if (argc != 3) {
            std::cerr << "Usage: " << argv[0] << " <netport> <serialport>\n";
            return 1;
        }

        boost::asio::io_context io_context;
        SerialInterface interface {io_context, argv[2], 115200};
        SCDTRServer s(io_context, std::atoi(argv[1]));
        s.bind_serial_interface(&interface);

        io_context.run();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}