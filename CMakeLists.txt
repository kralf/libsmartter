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
  HOME "http://aslforge.ethz.ch/projects/libsmartter"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX smartter-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  libsmartter libelrob
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  libsmartter libelrob-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
