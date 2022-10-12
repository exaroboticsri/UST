#!/bin/bash

# ADD python shebang to main.py and make it executable
FILE=./main.py
FIRSTLINE=$(head -n 1 "$FILE")
SHEBANG="#!/usr/bin/env python3"

if test -f "$FILE"; 
then
	echo -n "Try adding shebang to $FILE..."
	if [ "$FIRSTLINE" == "$SHEBANG" ]; 
	then
		echo "already exits!";
	else
		sed -i '1i'"$SHEBANG"'\' $FILE;
		echo "DONE!";
	fi
fi

echo -n "Try making $FILE executable..."
if [ -x "$FILE" ];
then 
	echo "already executable!"; 
else 
	$(chmod +x "$FILE")
	echo "DONE!"; 
fi
