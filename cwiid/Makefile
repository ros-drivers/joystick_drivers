all: installed

SVN_DIR = build/cwiid-svn
SVN_URL = http://abstrakraft.org/cwiid/svn/trunk
SVN_REVISION = -r201
SVN_PATCH =  cwiid-201.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && autoreconf -i -s
	cd $(SVN_DIR) && ./configure --disable-ldconfig --prefix=$(CURDIR)/cwiid
	cd $(SVN_DIR) && make $(ROS_PARALLEL_JOBS) && make install
	mkdir -p $(CURDIR)/src
	cp -R $(CURDIR)/cwiid/lib/python*/site-packages/* $(CURDIR)/src
	touch installed

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf $(CURDIR)/src $(CURDIR)/cwiid installed

wipe: clean
	rm -rf build patched

.PHONY : clean download
