#ifndef BASE_SESSION_H
#define BASE_SESSION_H

#include <boost/asio.hpp>
#include <deque>

class Session : public std::enable_shared_from_this<Session> {
public:
    Session(boost::asio::ip::tcp::socket socket) : socket(std::move(socket)) {}

    void start()
    {
        do_read();
    }

protected:
    virtual std::string on_read(char buf[1024]) = 0;

private:
    void do_read()
    {
        auto self(shared_from_this());
        socket.async_read_some(boost::asio::buffer(data_, max_length-1), [this, self](boost::system::error_code ec, std::size_t length) {
            if (!ec) {
                data_[length] = '\0';
                std::string response = on_read(data_);
                if (response.length() > 0) {
                    for (int i = 0; i < (response.length() - 1) / max_length + 1; i++) {
                        size_t end = max_length*(i+1) < response.length() ? max_length*(i+1) : response.length();
                        size_t sz = end - max_length*i;
                        messages.push_back(response.substr(max_length*i, sz));
                    }
                }
                if (!messages.empty())
                    do_write();
                else
                    do_read();
            }
        });
    }

    void do_write()
    {
        auto self(shared_from_this());
        boost::asio::async_write(socket, boost::asio::buffer(messages.front().data(), messages.front().length()), [this, self](boost::system::error_code ec, std::size_t) {
            if (!ec) {
                messages.pop_front();
                if (!messages.empty()) {
                    do_write();
                }
                else
                    do_read();
            }
        });
    }

    boost::asio::ip::tcp::socket socket;
    std::deque<std::string> messages;
    enum { max_length = 1024 };
    char data_[max_length];
};


#endif