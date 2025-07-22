#ifndef SESSION_H
#define SESSION_H

#include <optional>
#include "base_session.h"
#include "serial_interface.h"
#include "utils.h"

class SCDTRSession : public Session {
public:
    using Session::Session;

    void bind_serial_interface(SerialInterface *si, unsigned short id)
    {
        serial_interface = si;
        this->id = id;
        si->register_reader(shared_from_this());
    }

    void do_user_defined_task()
    {
        auto maybe_message = serial_interface->pop_message(id);
        if (maybe_message.has_value()) {
            messages.push_back(maybe_message.value() + '\n');
            std::cout << "[SESSION_" << id << "_OUT] " << (RawString) maybe_message.value() << std::endl;
            do_write();
        }
    }

protected:
    SerialInterface *serial_interface;
    unsigned short id;

    std::optional<std::string> on_read(const std::string& buf)
    {
        std::cout << "[SESSION_" << id << "_IN] " << (RawString) buf << std::endl;
        serial_interface->enqueue_write(buf, id);
        return std::nullopt;
    }
};

#endif