#!/bin/bash
set -e

git submodule update --init --recursive

case "$PLATFORM" in
"osx")
    wget -4 http://lamestation.net/downloads/travis/qt5.5.0-mac-clang.tgz
    tar xzf qt5.5.0-mac-clang.tgz
    mv local/ /Users/travis/local/
    ;;
"linux")
    ;;
*)
    echo "Invalid PLATFORM"
    exit 1
    ;;
esac

# packthing installation

case "$PLATFORM" in
"linux")
    pushd $HOME
    git clone https://github.com/lamestation/packthing
    pushd packthing
    sudo pip install -r requirements.txt
    sudo python setup.py install
    popd
    popd
    ;;
"osx")
    mkdir -p $HOME/local/lib/python2.7/site-packages
    easy_install --prefix=$HOME/local pyyaml
    pushd $HOME
    git clone https://github.com/lamestation/packthing
    pushd packthing
    python setup.py install --prefix=$HOME/local
    popd
    popd
    ;;
*)
    echo "Invalid PLATFORM"
    exit 1
    ;;
esac
