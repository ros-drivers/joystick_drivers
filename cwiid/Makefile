all: installed

#SVN_DIR = build/cwiid-svn
#SVN_URL = http://abstrakraft.org/cwiid/svn/trunk
#SVN_REVISION = -r 201
#SVN_PATCH =  cwiid-201.patch
#include $(shell rospack find mk)/svn_checkout.mk

FILENAME = cwiid-svn-r201.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
SOURCE_DIR = build/cwiid-svn
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH = cwiid-201.patch
include $(shell rospack find mk)/download_unpack_build.mk


installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && autoreconf -i -s
	cd $(SOURCE_DIR) && ./configure --disable-ldconfig --prefix=$(CURDIR)/cwiid
	cd $(SOURCE_DIR) && make $(ROS_PARALLEL_JOBS) && make install
	mkdir -p $(CURDIR)/src
	cp -R $(CURDIR)/cwiid/lib/python*/site-packages/* $(CURDIR)/src
	touch installed

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(CURDIR)/src $(CURDIR)/cwiid installed

wipe: clean
	rm -rf build patched

wiped: Makefile
	make wipe
	touch wiped

.PHONY : clean download
