#ifndef BASE_CLIENT_H
#define BASE_CLIENT_H

#include <string>
#include <boost/asio.hpp>

class Client {
public:
    Client(boost::asio::io_context& ctx, const std::string& host, unsigned short port);
    
    bool spin();

protected:
    boost::asio::ip::tcp::resolver res;
    boost::asio::ip::tcp::socket sock;
    boost::asio::ip::basic_resolver_results<boost::asio::ip::tcp> resolve_results;
    
    virtual bool main_loop() = 0;

private:
    bool connect();
};

#endif