#!/bin/bash
set -e

case "$PLATFORM" in
"osx")
    qmake -v
    packthing -h

    packthing -j4 dmg
    mv build/staging/elev8-flightcontroller-*.dmg .
    ;;
"linux")
    qmake -v
    packthing -h

    packthing -j4 run
    mv build/staging/elev8-flightcontroller-*.run .

    fakeroot packthing -j4 deb
    mv build/staging/elev8-flightcontroller-*.deb .
    ;;
*)
    echo "Invalid PLATFORM"
    exit 1
    ;;
esac
