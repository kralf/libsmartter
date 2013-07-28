cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libsmartter
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Driver library for the SmartTer."
  AUTHOR "Pierre Lamon"
  AUTHOR "Sascha Kolski"
  AUTHOR "Francois Pomerleau"
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/libsmartter"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX smartter-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libelrob libcpc
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libsmartter libelrob-dev libcpc-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev libcpc-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBSMARTTER_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libelrob-dev libcpc-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBSMARTTER_GIT_REVISION
)
