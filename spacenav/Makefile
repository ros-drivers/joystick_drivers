all: installed

SVN_DIR = spacenav_svn
SVN_URL = https://spacenav.svn.sourceforge.net/svnroot/spacenav/trunk
SVN_REVISION = -r 91
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR)/spacenavd && ./configure --prefix=$(CURDIR)/spacenavd
	cd $(SVN_DIR)/spacenavd && make
	#cd $(SVN_DIR)/spacenavd && make && make install
	cd $(SVN_DIR)/libspnav && ./configure --prefix=$(CURDIR)/libspnav
	mkdir -p libspnav/lib libspnav/lib64 libspnav/include
	cd $(SVN_DIR)/libspnav && make && make install
	touch installed

clean:
	-make -C $(SVN_DIR)/spacenavd clean
	-make -C $(SVN_DIR)/libspnav clean
	-$(RM) -r libspnav spacenavd installed

wipe: clean
	-rm -rf $(SVN_DIR)
