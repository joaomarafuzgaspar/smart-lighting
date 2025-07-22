#ifndef BASE_SERVER_H
#define BASE_SERVER_H

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include "base_server.h"

class Server {
public:
    Server(boost::asio::io_context& io_context, short port)
        : acceptor(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)), ctx {io_context}
    {
        do_accept();
    }

protected:
    void do_accept()
    {
        acceptor.async_accept([this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket) {
            if (!ec) {
                create_session(ctx, std::move(socket));
            }
            do_accept();
        });
    }

    virtual void create_session(boost::asio::io_context& io_context, boost::asio::ip::tcp::socket socket) = 0;

    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::io_context& ctx;
};

#endif