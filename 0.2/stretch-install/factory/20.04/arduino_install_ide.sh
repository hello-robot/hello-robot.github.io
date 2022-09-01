#!/bin/bash
# https://pastebin.com/gT1jfNNq
# This script will install ARDUINO

#                                Install Arduino IDE                                #
#####################################################################################

#Variables
#

    arduino=0
    platform=""
    version=""
    directory=""
    download=""
    check_version=false
    check_directory=false

    if [ -n "$1" ]
        then
        case $1 in
            1 | -v) arduino=1
					shift
                    platform="$1"
                    shift
                    version="$1"
                    download="$version"-linux"$platform"
                    ;;
            2 | -d) arduino=2
                    shift
                    directory="$1"/arduino
                    ;;
            3 | -c) arduino=3
                    shift
                    platform="$1"
                    shift
                    version="$1"
                    download="$version"-linux"$platform"
                    shift
                    directory="$1"/arduino
                    ;;
            * )     arduino=4
            		clear
    cat << HELP_USAGE
START################################################################################
#                     Program writed for 85 characters lenght                       #
#####################################################################################
#   Usage: $0 [-v platform version] [-d directory] [-c platform version directory]  # 
#                                                                                   #
#   -v : If you know what version you want to install EX i.sh -v arm 1.8.2          #
#   -d : If you know where to install EX i.sh -d .                                  #
#   -c : If you know version and where EX i.sh -c arm 1.8.2 .                       #
#                                                                                   #
#   If no parameter is chosen, the program will ask for them.                       #
#                                   (V)(;,,;)(V)                                    #
##################################################################################END
HELP_USAGE
					read -r -p"Press any key to close..." potato
					clear
					exit
        esac
    fi

	if [ "$arduino" != 3 ]
    	then
    	case $arduino in
            0 )     check_version=true
                    check_directory=true
                    ;;
            1 )     check_directory=true
                    ;;
            2 )     check_version=true
                    ;;
            * )		;;
    	esac
	fi

	if [ "$check_version" = "true" ]
    	then
		clear
    	cat << ARDUINO_PLATFORM
#####################################################################################

Which plataform are you using?
[1] ARM
[2] 32
[3] 64

#####################################################################################
ARDUINO_PLATFORM
        read -r platformid

        case $platformid in
        1 )     platform=arm
                ;;
        2 )     platform=32
                ;;
        3 )     platform=64
                ;;
        * )     echo "Please, select a valid platform"
                exit
                ;;
        esac
		clear

        cat << ARDUINO_VERSION
#####################################################################################

Introduce the version number X.X.X or nightly

#####################################################################################
ARDUINO_VERSION
		read -r version
		download="${version}-linux${platform}"
	fi

	if [ "$check_directory" = "true" ]
		then
		clear
		echo "#####################################################################################"

		echo "Select directory to install arduino"

		echo "#####################################################################################"
		read -r rDirectory
		directory="$rDirectory"/arduino
	fi  

	clear
		echo "Creating directory on \"$directory\" ..."
    mkdir "$directory" -p > /dev/null 
    	echo "Downloading Arduino $version for $platform."
    wget https://downloads.arduino.cc/arduino-"$download".tar.xz -q --show-progress -P "$directory"
    	echo "Download completed."
    	echo "Extracting, this might take a while..."
    tar xJf "$directory"/arduino-"$download".tar.xz -C "$directory" --strip=1
    	echo "Task completed."
    	echo "Deleting compressed file."
    rm "$directory"/arduino-"$download".tar.xz > /dev/null

    #read -r -p"Installation complete press any key to close Arduino IDE installer..." potato
    #clear
	return
