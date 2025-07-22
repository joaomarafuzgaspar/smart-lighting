#ifndef SERVER_H
#define SERVER_H

#include "base_server.h"
#include "serial_interface.h"
#include "session.h"

class SCDTRServer : public Server {
public:
    using Server::Server;
    void bind_serial_interface(SerialInterface* si)
    {
        serial_interface = si;
    }

protected:
    SerialInterface* serial_interface;

    void create_session(boost::asio::io_context& io_context, boost::asio::ip::tcp::socket socket)
    {
        unsigned short id = socket.remote_endpoint().port();
        auto sess = std::make_shared<SCDTRSession>(io_context, std::move(socket));
        sess->bind_serial_interface(serial_interface, id);
        sess->start();
    }
};

#endif