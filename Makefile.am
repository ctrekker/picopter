SUBDIRS = src ggk

bin_PROGRAMS = picopter
picopter_SOURCES = main.cpp
picopter_LDADD = src/libpicopter.a ggk/src/libggk.a -lpigpio -lpthread
picopter_LDLIBS = $(GLIB_LIBS) $(GIO_LIBS) $(GOBJECT_LIBS)
picopter_CPPFLAGS = -I$(srcdir)/eigen

AM_CPPFLAGS = -DONLY_WITH_BUILDSYSTEM
