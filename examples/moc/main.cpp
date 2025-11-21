#include <iostream>

#define MOC_IMPLEMENTATION
#include "games/moc.hpp"


int main()
{
    auto env{moc::Env()};
    bool terminated{false};

    while (!terminated) {
        auto sr{env.step(moc::ACT_ACCEL)};
        if (sr.terminated) terminated = true;

        std::cout << "reward " << sr.reward << "\n";
    }

    return 0;
}
