remake_add_library(smartter PREFIX OFF LINK ${LIBCPC_LIBRARIES}
  ${LIBELROB_LIBRARIES})
remake_add_headers()
remake_pkg_config_generate(REQUIRES libelrob libcpc)
