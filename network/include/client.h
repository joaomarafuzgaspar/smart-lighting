#ifndef CLIENT_H
#define CLIENT_H

#include "base_client.h"

class SCDTRClient : public Client {
public:
    using Client::Client;

protected:
    bool main_loop();
};

#endif