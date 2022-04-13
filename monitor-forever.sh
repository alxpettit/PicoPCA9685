#!/usr/bin/env bash

while true; do
    platformio run -t monitor || sleep 0.5
    sleep 0.5
done
