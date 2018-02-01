# Copyright (C) 2006 Ted Williams (wa0eir) <wa0eir@mchsi.com>
#  
# This file is free software; as a special exception the author gives
# unlimited permission to copy and/or distribute it, with or without 
# modifications, as long as this notice is preserved.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY, to the extent permitted by law; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.


##EXTRA_DIST   = src/psk31lx.ini src/psk31.cod

SUBDIRS      = src

install-data-hook:
	echo Installing data files to $(DESTDIR)$(pkgdatadir)
	$(INSTALL) -d $(DESTDIR)$(pkgdatadir)
	$(INSTALL_DATA) src/psk31.cod $(DESTDIR)$(pkgdatadir)
	$(INSTALL_DATA) src/psk31lx.ini $(DESTDIR)$(pkgdatadir)

	$(INSTALL) -d $(DESTDIR)@docdir@
	$(INSTALL_DATA) THANKS ChangeLog README $(DESTDIR)@docdir@


uninstall-local:
	echo Removing package data directory
	rm -rf $(DESTDIR)$(pkgdatadir)
	rm -rf $(DESTDIR)@docdir@


