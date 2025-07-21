#ifndef SESSION_H
#define SESSION_H

#include "base_session.h"
#include "serial_interface.h"

class SCDTRSession : public Session {
public:
    using Session::Session;

    void bind_serial_interface(SerialInterface *si)
    {
        serial_interface = si;
    }

protected:
    SerialInterface *serial_interface;

    std::string on_read(char buf[1024])
    {
        return "Test\n";
    }
};

#endif