#!/bin/bash

mkdir -p data/full/
cp /scratch2/data/iiswc-ae/iiswc-ae-hil-data.zip data/full/
unzip -o data/full/iiswc-ae-hil-data.zip -d data/full/
rm data/full/iiswc-ae-hil-data.zip