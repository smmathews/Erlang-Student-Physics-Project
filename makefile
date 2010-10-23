# makefile for PHY350 Spring 2008 by Shane M Mathews
# All content (C) 2008 DigiPen (USA) Corporation, all rights reserved.

all:
	(cd vecMath; $(MAKE))
	(cd graphics; $(MAKE))
	(cd physics; $(MAKE))
#	(cd GameEngine; $(MAKE))
	(cd application; $(MAKE))

clean:
	(cd vecMath; $(MAKE) clean)
	(cd graphics; $(MAKE) clean)
	(cd physics; $(MAKE) clean)
#	(cd GameEngine; $(MAKE) clean)
	(cd application; $(MAKE) clean)

#
# Build installer for Windows.
#
#win32: all
#	(cd win32; $(MAKE))
#	win32/make_installer
