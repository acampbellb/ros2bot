#!/usr/bin/env bash

PYTHON3_VERSION=`python3 -c 'import sys; version=sys.version_info[:3]; print("{0}.{1}".format(*version))'`

echo "Python3 version:  $PYTHON3_VERSION"
