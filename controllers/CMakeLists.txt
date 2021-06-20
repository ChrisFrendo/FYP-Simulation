add_library(basic_controller MODULE basic_controller.h basic_controller.cpp)
target_link_libraries(basic_controller
        basic_loop_functions
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)
