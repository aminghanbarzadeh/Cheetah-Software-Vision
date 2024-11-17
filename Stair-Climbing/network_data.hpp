#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "network_data.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const network_data* msg)
        {
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  delta_steps   = %f\n", (long long)msg->delta);

        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("stair_rise", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}