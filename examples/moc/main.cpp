#include <vector>
#define MOC_IMPLEMENTATION
#include "games/moc.hpp"

struct POMDPSolver
{
    struct StateNode;

    struct ActionNode
    {
        std::vector<StateNode> children;
        moc::Action act;
    }; // struct ActionNode

    struct StateNode
    {
        std::vector<moc::Obstacle> obs;
        std::vector<ActionNode> children;
        moc::Vec2f ego_position;
        moc::Vec2f ego_heading;
        moc::Vec2f ego_speed;
    }; // struct StateNode

    moc::Action solve(const StateNode& state);
private:
    StateNode root_;
}; // struct POMDPSolver


moc::Action POMDPSolver::solve(const POMDPSolver::StateNode& state)
{
    return moc::ACT_ACCEL;
}


int main()
{
    auto settings{moc::EnvSettings::from_default()};
    settings.num_persons = 200;
    auto env{moc::Env(settings)};
    bool terminated{false};

    POMDPSolver solver{};
    POMDPSolver::StateNode state{};
    bool first_step{true};
    while (!terminated) {
        const auto act{first_step ? moc::ACT_ACCEL : solver.solve(state)};
        const auto sr{env.step(act)};
        if (sr.terminated) terminated = true;
        first_step = false;
    }

    return 0;
}
