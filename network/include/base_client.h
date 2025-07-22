#ifndef BASE_CLIENT_H
#define BASE_CLIENT_H

#include <string>
#include <deque>
#include <boost/asio.hpp>

class Client {
public:
    Client(boost::asio::io_context& ctx, const std::string& host, unsigned short port)
        : ctx {ctx}, res {ctx}, sock {ctx}, read_buffer(1024)
    {
        resolve_results = res.resolve(host, std::to_string(port));
    }
    
    bool spin()
    {
        if (!connect())
            return false;

        do_socket_read();
        main_loop();

        ctx.run();
        return true;
    }

protected:
    boost::asio::io_context& ctx;
    boost::asio::ip::tcp::resolver res;
    boost::asio::ip::tcp::socket sock;
    boost::asio::ip::basic_resolver_results<boost::asio::ip::tcp> resolve_results;
    std::vector<char> read_buffer;
    std::deque<std::string> write_messages;
    
    virtual void main_loop() = 0;
    virtual void on_read(const std::string&) = 0;
    void write_message(const std::string& msg)
    {
        write_messages.push_back(msg);
        do_socket_write();
    }

private:
    bool connect()
    {
        boost::system::error_code ec;
        boost::asio::connect(sock, resolve_results, ec);
        if (ec)
            return false;
        return true;
    }

    void do_socket_read()
    {
        boost::asio::async_read_until(
            sock,
            boost::asio::dynamic_buffer(read_buffer),
            '\n',
            [this](const boost::system::error_code& ec, std::size_t) {
                if (!ec) {
                    std::string contents {read_buffer.begin(), read_buffer.end()};
                    read_buffer.clear();
                    
                    on_read(contents);

                    do_socket_read();
                }
            }
        );
    }

    void do_socket_write()
    {
        boost::asio::async_write(sock, boost::asio::buffer(write_messages.front().data(), write_messages.front().length()), [this](boost::system::error_code ec, std::size_t) {
            if (!ec) {
                write_messages.pop_front();
            }
        });
    }
};


#endif