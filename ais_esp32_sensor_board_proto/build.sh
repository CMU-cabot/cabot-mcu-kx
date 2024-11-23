#!/bin/bash

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function help() {
    echo "$0 [options] [build|upload|all]"
    echo ""
    echo "build      build the code (default action)"
    echo "upload     upload the code"
    echo "all        build and upload if build is success"
    echo ""
    echo "-h         show this help"
    echo "-b <board> set board (default=esp32:esp32:esp32)"
    echo "-p <port>  set port (default=/dev/ttyESP32)"
}

: ${ARDUINO_BOARD:="esp32:esp32:esp32"}
: ${ARDUINO_PORT:="/dev/ttyESP32"}

board=$ARDUINO_BOARD
port=$ARDUINO_PORT

while getopts "hb:p:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	b)
	    board=$OPTARG
	    ;;
	p)
	    port=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

target=$1
if [ -z $target ]; then
    target=build
fi

function build() {
    echo "building..."
    echo "arduino-cli compile -b $board ."
    arduino-cli compile -b $board .

    if [ $? -ne 0 ]; then
	err "Please check board ($board)"
    fi
}

function upload() {
    echo "uploading..."
    arduino-cli upload -b $board -p $port .

    if [ $? -ne 0 ]; then
	err "Please check board ($board) or port ($port)"
    fi
}    

if [ $target == "build" ]; then
    build
fi

if [ $target == "upload" ]; then
    upload
fi

if [ $target == "all" ]; then
    build && upload
fi
