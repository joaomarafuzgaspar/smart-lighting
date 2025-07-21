#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <sstream>
#include <list>
#include <boost/asio.hpp>

bool starts_with(const std::string& str, const std::string& prefix)
{
    return str.substr(0, prefix.size()) == prefix;
}

class SerialInterface {
public:
    SerialInterface(boost::asio::io_context& ctx, const std::string& port, unsigned int baudrate)
        : serial_port {ctx, port}
    {
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        do_read();
    }

    std::string get_message()
    {
        return messages.back();
    }

    std::string get_message(uint8_t id)
    {
        for (auto i = messages.begin(); i != messages.end(); i++) {
            if (starts_with(*i, std::to_string(id))) 
                return *i;
        }
        return "";
    }

    std::string pop_message()
    {
        std::string result = messages.back();
        messages.pop_back();
        return result;
    }

    std::string pop_message(uint8_t id)
    {
        std::string result {};
        for (auto i = messages.begin(); i != messages.end(); i++) {
            if (starts_with(*i, std::to_string(id))) {
                result = *i;
                messages.erase(i);
                break;
            }
        }
        return result;
    }

private:
    boost::asio::serial_port serial_port;
    std::list<std::string> messages;
    std::stringstream ss;
    char buffer[1024];

    void do_read()
    {
        std::cout << "Do read!" << std::endl;
        serial_port.async_read_some(boost::asio::buffer(buffer, sizeof(buffer)-1), [this](boost::system::error_code ec, std::size_t length) {
            std::cout << "Code: " << ec.message() << std::endl;
            if (!ec) {
                buffer[length] = '\0';
                std::string received {buffer}, temp;
                std::cout << "Received: " << received << std::endl;
                std::size_t loc = received.find('\n');
                while (loc != std::string::npos) {
                    temp = received.substr(0, loc);
                    ss << temp;
                    messages.push_back(ss.str());
                    ss.str("");
                    received = temp;
                    loc = received.find('\n');
                }
                ss << received;
            }
            do_read();
        });
    }
};

#endif