idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       LDFRAGMENTS "linker.lf")

# Motor selection (see motor_params.hpp). Default: the NineBot on the dyno.
#   idf.py -DATOS_MOTOR=hub20 build
if(ATOS_MOTOR STREQUAL "hub20")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_MOTOR_HUB20)
elseif(ATOS_MOTOR STREQUAL "flatbot")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_MOTOR_FLATBOT)
elseif(ATOS_MOTOR STREQUAL "drone")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_MOTOR_DRONE)
endif()

# Bus voltage override for profiles that honour it (drone): idf.py -DATOS_VBUS=12 build
if(DEFINED ATOS_VBUS)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_VBUS=${ATOS_VBUS})
endif()

# Core for the sampling ISR (see set_edge_on_core): idf.py -DATOS_SAMPLER_CORE=1 build
if(DEFINED ATOS_SAMPLER_CORE)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_SAMPLER_CORE=${ATOS_SAMPLER_CORE})
endif()
