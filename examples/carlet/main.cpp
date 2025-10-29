// #define CARLET_IMPLEMENTATION
#include "carlet.hpp"
#include <cstdlib>
#include <ctime>


int main()
{
    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x = 0.0f, .y = 0.0f, .z=0.0f},
        50.0f, 3, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.roads.push_back(straight_road);
    const int car_id{sim->create_ctrl_veh(carlet::veh_model::tesla)};
    sim->gen_random_vehs(10);

    while (sim->is_running()) {
        sim->get_ctrl_veh(car_id).act(0.0f, 0.5f);
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}
