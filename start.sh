#!/bin/bash

docker compose build 

docker compose up &

sleep 2s

docker exec -i robot_docker-px4-1 bash -c 'mavlink-routerd &'

docker exec -i robot_docker-px4-1 bash -c 'source ~/.bashrc && bridge &'


