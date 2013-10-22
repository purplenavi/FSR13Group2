#!/bin/sh
for z in hiekkalaatikko/* ; do
    cd $z
    make
    cd -
done
