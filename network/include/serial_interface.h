#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#define MAX_READ_MESSAGES_SIZE 6000

#include <list>
#include <sstream>
#include <vector>
#include <boost/asio.hpp>
#include "base_session.h"
#include "utils.h"

class SerialInterface {
public:
    SerialInterface(boost::asio::io_context& ctx, const std::string& port, unsigned long baudrate)
        : serial_port {ctx, port}
    {
        buffer.resize(1024);
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port.set_option(boost::asio::serial_port_base::character_size(8));
        serial_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
        do_read();
    }

    static bool is_identified_message(const std::string &message) {
        return message.length() > 0 && message[0] >= '0' && message[0] <= '9';
    }

    std::optional<std::string> get_message()
    {
        if (read_messages.size() == 0)
            return std::nullopt;
        return read_messages.back();
    }

    std::optional<std::string> get_message(unsigned short id)
    {
        for (auto i = read_messages.begin(); i != read_messages.end(); i++) {
            if (starts_with(*i, std::to_string(id))) 
                return *i;
        }
        return std::nullopt;
    }

    std::optional<std::string> pop_message()
    {
        if (read_messages.size() == 0)
            return std::nullopt;
        std::string result = read_messages.back();
        read_messages.pop_back();
        return result;
    }

    std::optional<std::string> pop_message(unsigned short id)
    {
        std::string result {};
        std::string prefix {std::to_string(id)};
        prefix += " ";
        for (auto i = read_messages.begin(); i != read_messages.end(); i++) {
            if (starts_with(*i, prefix)) {
                result = i->substr(prefix.length());
                read_messages.erase(i);
                return result;
            }
        }
        return std::nullopt;
    }

    void enqueue_write(const std::string& msg)
    {
        write_messages.push_back(msg);
        do_write();
    }

    void enqueue_write(const std::string& msg, unsigned short id)
    {
        std::stringstream builder;
        builder << id << " " << msg;
        write_messages.push_back(builder.str());
        do_write();
    }

    void register_reader(const std::weak_ptr<Session>& reader)
    {
        registered_readers.push_back(reader);
    }

private:
    boost::asio::serial_port serial_port;
    std::list<std::string> read_messages {};
    std::list<std::string> write_messages {};
    std::vector<std::weak_ptr<Session>> registered_readers {};
    std::stringstream ss;
    std::vector<char> buffer;

    void add_message(const std::string& message) {
        std::vector<unsigned short> ids; 
        size_t next = 0, last = 0;
        std::string token;
        while ((next = message.find(' ', last)) != std::string::npos) {
            token = message.substr(last, next-last);
            unsigned short id = (unsigned short) std::strtoul(token.c_str(), NULL, 10);
            if (id == 0)
                break;
            ids.push_back(id);
            last = next + 1;
        }
        
        std::string message_without_ids {message.substr(last)};
        for (unsigned short id : ids) {
            read_messages.push_back(std::to_string(id) + " " + message_without_ids);
            if (read_messages.size() > MAX_READ_MESSAGES_SIZE)
                read_messages.pop_front();
        }
    }

    void notify_readers()
    {
        for (auto it = registered_readers.begin(); it != registered_readers.end();) {
            auto shared_ref = it->lock();
            if (!shared_ref)
                it = registered_readers.erase(it);
            else {
                shared_ref->do_user_defined_task();
                it++;
            }
        }
    }

    void do_read()
    {
        serial_port.async_read_some(
            boost::asio::buffer(buffer), 
            [this](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    buffer[length] = '\0';
                    std::string received {buffer.data()}, temp;
                    std::size_t loc = received.find("\r\n");
                    bool alternative = false;
                    if (loc == std::string::npos) {
                        loc = received.find('\n');
                        alternative = true;
                    }
                    while (loc != std::string::npos) {
                        temp = received.substr(0, loc);
                        ss << temp;

                        std::string message {ss.str()};
                        if (is_identified_message(message))
                            add_message(message);

                        std::cout << "[SERIAL_IN] " << (RawString) message << std::endl;
                        ss.str("");
                        received = received.substr(loc+2-alternative);
                        if (!alternative)
                            loc = received.find("\r\n");
                        else
                            loc = received.find('\n');
                    }
                    ss << received;
                    notify_readers();
                    do_read();
                }
            }
        );
    }

    void do_write()
    {
        boost::asio::async_write(serial_port, boost::asio::buffer(write_messages.front().data(), write_messages.front().length()), [this](boost::system::error_code ec, std::size_t) {
            std::cout << "[SERIAL_OUT] " << (RawString) write_messages.front() << std::endl;
            write_messages.pop_front();
        });

    }
};

#endif