#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_INIT([psk31lx], [1.2], [wa0eir@mchsi.com])
AC_CONFIG_SRCDIR([src])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([psk31lx], [1.2])
AC_CONFIG_SRCDIR([config.h.in])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for libraries.
AC_CHECK_LIB([m], [sqrt], [],
             [AC_MSG_ERROR("ERROR: math library was not found")])

AC_CHECK_LIB([pthread], [pthread_create], [],
             [AC_MSG_ERROR("ERROR: pthread library was not found")])

AC_CHECK_LIB([curses], [initsrc], []
             [AC_MSG_ERROR("ERROR: ncurses library was not found")])


# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h netinet/in.h stdlib.h string.h sys/ioctl.h sys/socket.h sys/time.h termios.h unistd.h ncurses.h curses.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE
AC_HEADER_TIME
AC_STRUCT_TM

# Checks for library functions.
AC_PROG_GCC_TRADITIONAL
AC_FUNC_MALLOC
AC_FUNC_SELECT_ARGTYPES
AC_CHECK_FUNCS([atexit bzero memset pow select sqrt strdup])

AC_CONFIG_FILES([Makefile
                 src/Makefile])
AC_OUTPUT
