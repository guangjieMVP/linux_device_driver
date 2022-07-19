#!/bin/sh

sudo busybox mount -o nolock -t nfs 192.168.1.106:/home/ares/work/nfs_root /home/debian/nfs_rootfs
