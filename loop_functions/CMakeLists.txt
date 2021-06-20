 link_directories(${CMAKE_BINARY_DIR}/controllers)
 set(basic_loop_functions_SOURCES basic_loop_functions.cpp)

  if(ARGOS_COMPILE_QTOPENGL)
    set(basic_loop_functions_SOURCES
      ${basic_loop_functions_SOURCES}
      basic_qtuser_functions.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)

 add_library(basic_loop_functions SHARED ${basic_loop_functions_SOURCES})
 target_link_libraries(basic_loop_functions
         argos3core_simulator
         argos3plugin_simulator_dynamics2d
         argos3plugin_simulator_entities
         argos3plugin_simulator_footbot
         argos3plugin_simulator_genericrobot
         argos3plugin_simulator_media)

 if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(basic_loop_functions argos3plugin_simulator_qtopengl)
 endif (ARGOS_COMPILE_QTOPENGL)
