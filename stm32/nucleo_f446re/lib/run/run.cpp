#include "run.hpp"

namespace run {
void setup()
{

}
void loop()
{

}
}

extern "C" void setup_c()
{
    run::setup();
}

extern "C" void loop_c()
{
    run::loop();
}
