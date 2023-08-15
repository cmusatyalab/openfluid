#!/bin/bash
args=$*
/usr/bin/nvidia-smi -a
python3.8 ./main.py $args