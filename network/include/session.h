#ifndef SESSION_H
#define SESSION_H

#include "base_session.h"
#include "serial_interface.h"

class SCDTRSession : public Session {
public:
    using Session::Session;

    void bind_serial_interface(SerialInterface *si, unsigned short id)
    {
        serial_interface = si;
        this->id = id;
    }

protected:
    SerialInterface *serial_interface;
    unsigned short id;

    std::string on_read(const std::string& buf)
    {
        serial_interface->enqueue_write(buf, id);
        return "Received\n";
    }

    void do_user_defined_task()
    {
        auto maybe_message = serial_interface->pop_message(id);
        if (maybe_message.length() > 0) {
            std::cout << "Got message" << std::endl;
            messages.push_back(maybe_message);
        }
    }
};

#endif