bin_PROGRAMS = picopter
picopter_SOURCES = main.cpp
picopter_LDADD = src/libpicopter.a
picopter_CPPFLAGS = -I$(srcdir)/eigen

SUBDIRS = src ggk

AM_CPPFLAGS = -DONLY_WITH_BUILDSYSTEM
