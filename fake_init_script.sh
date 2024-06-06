#!/bin/sh
for i in $(seq 1 250);
do
    echo "Looping inside a shell script: $i"
    sleep 1
done
