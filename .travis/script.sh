#!/bin/bash
set -e

case "$PLATFORM" in
"osx")
    qmake -v
    packthing -h

    packthing -j4 dmg
    mv build/staging/elev8-groundstation-*.dmg .
    ;;
"linux")
    qmake -v
    packthing -h

    packthing -j4 run
    mv build/staging/elev8-groundstation-*.run .

    fakeroot packthing -j4 deb
    mv build/staging/elev8-groundstation-*.deb .
    ;;
*)
    echo "Invalid PLATFORM"
    exit 1
    ;;
esac
