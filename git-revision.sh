#!/bin/sh

#Must first create a tag like this:
#git tag generic-stm32f4

revisioncount=`git log --oneline | wc -l | tr -d ' '`
projectversion=`git describe --tags --long`
cleanversion=${projectversion%%-*}

echo "$projectversion-$revisioncount"
