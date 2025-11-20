#include "carlet.hpp"

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    srand(time(NULL));

    const auto straight_road{carlet::Road::gen_straight(
        Vector3{.x=0.0f, .y=0.0f, .z=0.0f},
        Vector3{.x=2000.0f, .y=0.0f, .z=0.0f},
        2, 3.7f)};

    auto sim{carlet::Simulator::instance()};
    sim->map().road_net.push_back(straight_road);
    sim->create_ctrl_veh(carlet::veh_model::tesla, 1);
    sim->gen_random_vehs(20,
        carlet::kmph_to_mps(40.0),
        carlet::kmph_to_mps(80.0));

    carlet::Veh* v{};
    carlet::Veh::AngleControl ctrl{.eps_angle=carlet::deg_to_rad(30.0f), .accel = 2.0f};

    while (sim->is_running()) {
        if ((v = sim->get_ctrl_veh()) != nullptr) {
            v->act(ctrl);
        }
        sim->step(0.02f);
        sim->render();
    }
    return 0;
}