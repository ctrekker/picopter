AC_INIT([picopter], [1.0])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CHECK_LIB([pthread], pthread_create, [], [AC_MSG_ERROR([pthread library not found])])
AC_CHECK_LIB([glib-2.0], g_variant_new, [], [AC_MSG_ERROR([glib-2.0 library not found])])
AC_CHECK_LIB([gio-2.0], g_dbus_method_invocation_return_dbus_error, [], [AC_MSG_ERROR([gio-2.0 library not found])])
AC_CHECK_LIB([gobject-2.0], g_object_unref, [], [AC_MSG_ERROR([gobject-2.0 library not found])])
AC_PROG_CXX
AC_PROG_RANLIB
AM_PROG_AR
AX_CXX_COMPILE_STDCXX_17

AC_SUBST(GLIB_CFLAGS)
AC_SUBST(GIO_CFLAGS)
AC_SUBST(GOBJECT_CFLAGS)

AC_CONFIG_SUBDIRS([
 src
 ggk
])
AC_CONFIG_FILES([
 Makefile
 src/Makefile
])
AC_OUTPUT
