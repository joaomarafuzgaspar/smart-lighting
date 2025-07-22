#include <cstdlib>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

#include "serial_interface.h"
#include "server.h"
#include "session.h"

int main(int argc, char* argv[])
{
    try {
        std::string host {"127.0.0.1"}, serialport;
        unsigned short port = 5555;
        unsigned long baudrate = 115200;

        boost::program_options::options_description desc("Usage:");
        desc.add_options()
            ("help", "produce help message")
            ("host", boost::program_options::value<std::string>()->default_value(host), "set the host to bind")
            ("port", boost::program_options::value<unsigned short>()->default_value(port), "set the networking port to bind")
            ("serialport", boost::program_options::value<std::string>(), "set the serial port")
            ("baudrate", boost::program_options::value<unsigned long>()->default_value(baudrate), "set the baudrate");

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if (vm.count("host"))
            host = vm["host"].as<std::string>();
        if (vm.count("port"))   
            port = vm["port"].as<unsigned short>();
        if (vm.count("serialport"))
            serialport = vm["serialport"].as<std::string>();
        else {
            std::cerr << "Missing required argument '--serialport'" << std::endl;
            return 0;
        }
        if (vm.count("baudrate"))
            baudrate = vm["baudrate"].as<unsigned long>();

        boost::asio::io_context io_context;
        SerialInterface interface {io_context, serialport, baudrate};
        SCDTRServer s(io_context, port);
        s.bind_serial_interface(&interface);

        io_context.run();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}