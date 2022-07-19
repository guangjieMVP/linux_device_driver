#!/bin/sh

_do () {
         $@ || ( cp -rf /tmp/boot /; echo "kernel update failed: $@"; exit -1; )
}

if [ ! -f /boot/vmlinuz* ]; then
        echo "error:fire kernel no exit!"
else
        cp -rf /boot /tmp

   _do dpkg -r linux-image-$(uname -r)

        _do dpkg -i $1

        if [ -f /boot/vmlinuz* ]; then
                rm -rf /tmp/boot
        else
                cp -rf /tmp/boot /
        fi
fi
