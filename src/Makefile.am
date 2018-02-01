# Copyright (C) 2006 Ted Williams (wa0eir) <wa0eir@mchsi.com>
#  
# This file is free software; as a special exception the author gives
# unlimited permission to copy and/or distribute it, with or without 
# modifications, as long as this notice is preserved.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY, to the extent permitted by law; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

EXTRA_DIST      = psk31lx.ini psk31.cod 

AM_CXXFLAGS     = -Wall -Wno-write-strings -DUSE_PTHREAD \
                  -DPKG_DATA_DIR=\"$(pkgdatadir)\"

psk31lx_SOURCES = term-psk31.C text-window.C text-window.h server.C server.h \
                  psk31-coder.C psk31-coder.h psk31-fft.C psk31-fft.h \
                  psk31-receiver.C psk31-receiver.h \
                  psk31-transmitter.C psk31-transmitter.h \
                  hansis_fft.C hansis_fft.h coeff.h

bin_PROGRAMS    = psk31lx

psk31lx_LDADD   = $(LIBS) -lncurses -lm -lpthread

