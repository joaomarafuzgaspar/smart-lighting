#include <iostream>
#include "client.h"

int main()
{
    boost::asio::io_context io;
    SCDTRClient client {io, "127.0.0.1", 8000};
    client.spin();
}