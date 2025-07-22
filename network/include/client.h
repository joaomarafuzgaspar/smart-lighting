#ifndef CLIENT_H
#define CLIENT_H

#include "base_client.h"

class SCDTRClient : public Client {
public:
    SCDTRClient(boost::asio::io_context& ctx, const std::string& host, unsigned short port)
        : Client(ctx, host, port), input_stream {ctx, ::dup(STDIN_FILENO)}, input_buffer {1024}
    {}

protected:
    boost::asio::posix::stream_descriptor input_stream;
    boost::asio::streambuf input_buffer;

    void on_read(const std::string& str)
    {
        std::cout << str << std::endl;
    }

    void main_loop()
    {
        boost::asio::async_read_until(
            input_stream,
            input_buffer,
            '\n',
            [this](const boost::system::error_code& ec, std::size_t length) {
                if (!ec) {
                    std::istream input_stream(&input_buffer);
                    std::string line;
                    std::getline(input_stream, line);
                    
                    write_message(line + '\n');

                    main_loop();
                }
                else {
                    sock.shutdown(sock.shutdown_both);
                }
            }
        );
    }
};

#endif