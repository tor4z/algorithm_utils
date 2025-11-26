#define ROCK_SAMPLE_IMPLEMENTATION
#include "games/rock_sample.hpp"


int main()
{
    bool terminated{false};
    auto env{rs::Env()};

    rs::Action act{rs::ACT_RIGHT};
    while (!terminated) {
        const auto sr{env.step(act)};
        terminated = sr.terminated;
    }
    return 0;
}
