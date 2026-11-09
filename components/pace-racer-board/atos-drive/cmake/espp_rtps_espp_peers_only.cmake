# Builds espp/rtps's SPDPAgent.cpp so the wheel only takes peers that run espp (vendor id
# 13.37): the MIB, the other wheel, and bench stand-ins such as tests/host_mib/mib_stub.
#
# The wheel's topics are only for the MIB. Every ROS 2 process on the robot LAN is a
# CycloneDDS participant, and each took a slot in the wheel's peer table and its endpoint
# proxy pools. With enough of them the wheel either hung (espp/rtps < 1.4.1) or, with
# pools big enough to hold them, ran out of internal RAM (rammp-org/pace-racer-fw#22).
# Ignoring them keeps the profile's default pools, at 0.4.7's RAM, however many ROS 2
# processes run. ROS 2 tools can't reach a wheel directly; they go through the MIB.
#
# The source is rewritten at configure time, so managed_components stays untouched, and the
# build stops if a new espp/rtps changes this code.
idf_component_get_property(rtps_lib espp__rtps COMPONENT_LIB)
idf_component_get_property(rtps_dir espp__rtps COMPONENT_DIR)
set(spdp_src "${rtps_dir}/src/discovery/SPDPAgent.cpp")
set(spdp_out "${CMAKE_BINARY_DIR}/espp_rtps/SPDPAgent.cpp")

file(READ "${spdp_src}" spdp_text)
set(own_packet "    return; // Our own packet\n  }\n")
string(FIND "${spdp_text}" "void SPDPAgent::processProxyData() {\n  if (m_proxyDataBuffer.m_guid.prefix.id == mp_participant->m_guidPrefix.id) {\n${own_packet}" at)
if(at EQUAL -1)
  message(FATAL_ERROR "espp/rtps: SPDPAgent::processProxyData() has changed; "
                      "review cmake/espp_rtps_espp_peers_only.cmake")
endif()
string(REPLACE "${own_packet}" "${own_packet}  if (m_proxyDataBuffer.m_vendorId.vendorId != Config::VENDOR_ID.vendorId) {\n    return; // Not espp: a ROS 2 process, which the wheel has no use for\n  }\n" spdp_patched "${spdp_text}")
file(CONFIGURE OUTPUT "${spdp_out}" CONTENT "${spdp_patched}" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${spdp_src}")

get_target_property(rtps_sources ${rtps_lib} SOURCES)
list(TRANSFORM rtps_sources REPLACE ".*/src/discovery/SPDPAgent\\.cpp$" "${spdp_out}")
if(NOT "${spdp_out}" IN_LIST rtps_sources)
  message(FATAL_ERROR "espp/rtps: SPDPAgent.cpp isn't among the component's sources")
endif()
set_target_properties(${rtps_lib} PROPERTIES SOURCES "${rtps_sources}")
