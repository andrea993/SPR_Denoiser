#/bin/sh

cat $1 | awk '{print $3 " " $4}' | ./main.exe
