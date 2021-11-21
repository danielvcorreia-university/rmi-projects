#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="mapping.out"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
	cd C1; java jClient -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
	cd C2; python3 mainRob.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
    3)
        cd C3; python3 mainRob.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
esac

