#!/bin/bash
# loop.sh - run a half-assed logging loop for Powermon
# you might want to run this
#  nohup ... 2>&1 > /dev/null &
# scruss - 2014-10-18

# needs grabserial - https://github.com/tbird20d/grabserial
# and ts from gnu 'moreutils'

filebase='powermon'
folder='data'
currentfile="${filebase}-current.log"
# log for a day into one file
duration=86400
# TEST - five minutes
# duration=300

# device settings - don't change the speed, it's hardcoded
device='/dev/ttyACM0'
speed=38400

# log file timestamp format
# %FT%T%z gives 2014-10-19T12:36:38-0400 for example
# (wouldn't need this if grabserial had real time support)
timestamp='%FT%T%z'

while
    true
do
    stty -F $device cs8 $speed ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
    outfile="${folder}/${filebase}-$(date +%Y%m%d%H%M%S).log"
    if
	[ -f $outfile ]
    then
	echo "*** $outfile exists - quitting"
	exit 1
    fi
    rm -f $currentfile
    touch $outfile && ln -s $outfile $currentfile
    echo Logging to $outfile for $duration ...
    grabserial -d $device -b $speed -e $duration | ts "$timestamp" >> "$outfile"
done
