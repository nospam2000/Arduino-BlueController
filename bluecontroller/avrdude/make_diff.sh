#!/bin/sh
diff -x .DS_Store -x .vimrc -x '*.ods' -x Makefile.in -x 'Makefile.new*' -x '*RFComm*' -x '*RingBuffer*' -x '*.m' -x '*~' -x ac_cfg.h.in -x aclocal.m4 -x avrdude.spec -x configure -x avrdude.info -x version.texi -x stamp-vti -Naur avrdude-5.10.upstream/ avrdude-5.10.bt
