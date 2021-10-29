SUBDIRS = src ggk

bin_PROGRAMS = picopter
picopter_SOURCES = main.cpp
picopter_LDADD = src/libpicopter.a -lpigpio
picopter_CPPFLAGS = -I$(srcdir)/eigen

AM_CPPFLAGS = -DONLY_WITH_BUILDSYSTEM
