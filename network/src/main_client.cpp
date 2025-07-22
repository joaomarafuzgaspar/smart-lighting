#include <iostream>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

#include "client.h"

int main(int argc, char *argv[])
{
    try {
        std::string ip {"127.0.0.1"};
        unsigned short port = 5555;

        boost::program_options::options_description desc("Usage:");
        desc.add_options()
            ("help", "produce help message")
            ("ip", boost::program_options::value<std::string>()->default_value(ip), "set the server IP")
            ("port", boost::program_options::value<unsigned short>()->default_value(port), "set the server port");

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if (vm.count("ip"))
            ip = vm["ip"].as<std::string>();
        if (vm.count("port"))
            port = vm["port"].as<unsigned short>();

        boost::asio::io_context io;
        SCDTRClient client {io, ip, port};

        if (!client.spin())
            std::cerr << "Error connecting to server" << std::endl;
    }
    catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}