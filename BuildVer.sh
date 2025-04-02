#!/bin/bash
echo 'Updated Ver and build...'
time2=$(date "+%y%m%d%H%M")
echo 'ESP_3.2.'$time2 >version.txt
idf.py build
