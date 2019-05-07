#!/bin/bash

screen -dmS rosbridge roslaunch rosbridge_server rosbridge_websocket.launch
webfsd -F -p 9876
