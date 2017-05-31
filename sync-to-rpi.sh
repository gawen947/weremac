#!/bin/sh

rsync -avhrR --progress g3-plc pi@192.168.1.11:
rsync -avhrR --progress g3-plc pi@192.168.1.12:
