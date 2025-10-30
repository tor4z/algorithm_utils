#include <cstdlib>
#include <ctime>

#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


int main()
{
    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=5000.0f, .y=0.0f, .z=0.0f},
        3, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.roads.push_back(straight_road);
    const int car_idx{sim->create_ctrl_veh(carlet::veh_model::tesla)};
    sim->gen_random_vehs(10);

    carlet::Control ctrl{.steer=0.0f, .accel=0.5f};
    while (sim->is_running()) {
        carlet::Veh* v{};
        if ((v = sim->get_ctrl_veh(car_idx)) != nullptr) {
            v->act(ctrl);
        }
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}
