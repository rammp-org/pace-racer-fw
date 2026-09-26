idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       LDFRAGMENTS "linker.lf")

# Motor selection (see motor_params.hpp). Default: the NineBot on the dyno.
#   idf.py -DATOS_MOTOR=hub20 build
if(ATOS_MOTOR STREQUAL "hub20")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_MOTOR_HUB20)
elseif(ATOS_MOTOR STREQUAL "flatbot")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ATOS_MOTOR_FLATBOT)
endif()
