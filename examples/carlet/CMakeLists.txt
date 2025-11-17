add_executable(simple_planning
    simple_planning.cpp
)

target_link_libraries(simple_planning
    raylib
)


add_executable(planning_sim
    planning_sim.cpp
)

target_link_libraries(planning_sim
    raylib
)
