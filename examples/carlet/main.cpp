#define CARLET_IMPLEMENTATION
#include "carlet.hpp"


int main()
{
    auto sim{carlet::Simulator::instance()};
    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x = 0.0f, .y = 0.0f, .z=0.0f},
        50.0f, 2, 3.7f)};
    sim->map().road_net.roads.push_back(straight_road);

    const int car_id{sim->create_ctrl_veh()};
    sim->create_random_vehs(3);

    while (sim->is_running()) {
        sim->get_ctrl_veh(car_id).act(0.0f, 0.2f);
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}
