#!/bin/bash

image="pc_heightmap_ground_filter"
tag="latest"

docker build -t $image:$tag .