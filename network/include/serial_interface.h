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
        return read_messages.back();
    }

    std::string get_message(unsigned short id)
    {
        for (auto i = read_messages.begin(); i != read_messages.end(); i++) {
            if (starts_with(*i, std::to_string(id))) 
                return *i;
        }
        return "";
    }

    std::string pop_message()
    {
        std::string result = read_messages.back();
        read_messages.pop_back();
        return result;
    }

    std::string pop_message(unsigned short id)
    {
        std::string result {};
        std::string prefix {std::to_string(id)};
        prefix += " ";
        for (auto i = read_messages.begin(); i != read_messages.end(); i++) {
            if (starts_with(*i, prefix)) {
                result = i->substr(prefix.length());
                read_messages.erase(i);
                break;
            }
        }
        return result;
    }

    void enqueue_write(const std::string& msg)
    {
        write_messages.push_back(msg);
    }

    void enqueue_write(const std::string& msg, unsigned short id)
    {
        std::stringstream builder;
        builder << id << " " << msg;
        write_messages.push_back(builder.str());
    }

private:
    boost::asio::serial_port serial_port;
    std::list<std::string> read_messages {};
    std::list<std::string> write_messages {};
    std::stringstream ss;
    char buffer[1024];

    void do_read()
    {
        serial_port.async_read_some(boost::asio::buffer(buffer, sizeof(buffer)-1), [this](boost::system::error_code ec, std::size_t length) {
            if (!ec) {
                buffer[length] = '\0';
                std::string received {buffer}, temp;
                std::size_t loc = received.find("\r\n");
                while (loc != std::string::npos) {
                    temp = received.substr(0, loc);
                    ss << temp;
                    read_messages.push_back(ss.str());
                    std::cout << "Added to messages: '" << read_messages.back() << "'" << std::endl;
                    ss.str("");
                    received = received.substr(loc+2);
                    loc = received.find("\r\n");
                }
                ss << received;
            }
            if (!write_messages.empty())
                do_write();
            else
                do_read();
        });
    }

    void do_write()
    {
        serial_port.async_write_some(boost::asio::buffer(write_messages.front().data(), write_messages.front().length()), [this](boost::system::error_code ec, std::size_t) {
            if (!ec) {
                write_messages.pop_front();
                if (!write_messages.empty())
                    do_write();
                else
                    do_read();
            }
        });
    }
};

#endif