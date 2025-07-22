#ifndef BASE_SESSION_H
#define BASE_SESSION_H

#include <optional>
#include <boost/asio.hpp>
#include <deque>

class Session : public std::enable_shared_from_this<Session> {
public:
    Session(boost::asio::io_context& io_context, boost::asio::ip::tcp::socket socket) : ctx {io_context}, socket(std::move(socket)) {}

    void start()
    {
        do_read();
    }
    virtual void do_user_defined_task() = 0;

protected:
    boost::asio::io_context &ctx;
    std::deque<std::string> messages;

    virtual std::optional<std::string> on_read(const std::string&) = 0;

    void do_read()
    {
        auto self(shared_from_this());
        socket.async_read_some(boost::asio::buffer(data_, max_length-1), [this, self](boost::system::error_code ec, std::size_t length) {
            if (!ec) {
                data_[length] = '\0';
                auto maybe_response = on_read(data_);
                if (maybe_response.has_value() > 0) {
                    std::string response = maybe_response.value();
                    for (size_t i = 0; i < (response.length() - 1) / max_length + 1; i++) {
                        size_t end = max_length*(i+1) < response.length() ? max_length*(i+1) : response.length();
                        size_t sz = end - max_length*i;
                        messages.push_back(response.substr(max_length*i, sz));
                    }
                }
                if (!messages.empty())
                    do_write();
                else {
                    do_read();
                }
            }
        });
    }

    void do_write()
    {
        auto self(shared_from_this());
        boost::asio::async_write(socket, boost::asio::buffer(messages.front().data(), messages.front().length()), [this, self](boost::system::error_code ec, std::size_t) {
            if (!ec) {
                messages.pop_front();
                if (!messages.empty())
                    do_write();
                else
                    do_read();
            }
        });
    }

    boost::asio::ip::tcp::socket socket;
    enum { max_length = 1024 };
    char data_[max_length];
};


#endif