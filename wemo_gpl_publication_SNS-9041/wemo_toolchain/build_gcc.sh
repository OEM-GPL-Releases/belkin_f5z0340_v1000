#!/bin/sh

if [ ! -e /opt ]; then
    echo "sudo mkdir /opt"
    sudo mkdir /opt
fi

if [ ! -w /opt ]; then
    user=`whoami`
    echo "sudo chown $user /opt"
    sudo chown $user /opt
fi

tar xjf tarballs/backfire_10.03_source.tar.bz2
cd backfire_10.03
ln -s ../tarballs dl
tar xzf dl/backfire_mods.tgz
make prepare V=99

touch /opt/toolchain-mipsel_gcc-4.3.3+cs_uClibc-0.9.33.2/usr/include/linux/autoconf.h

cd /opt/toolchain-mipsel_gcc-4.3.3+cs_uClibc-0.9.33.2/usr/bin/
for file in mipsel-openwrt-linux-uclibc-* ; do
    link=`echo $file | sed s/-openwrt//`
    ln -sf $file $link
    link1=`echo $link | sed s/-uclibc//`
    ln -sf $file $link1
done
