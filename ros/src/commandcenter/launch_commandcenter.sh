#!/bin/bash

echo "starting Server on 192.168.1.132:9876"
cd ../Software/ros/src/commandcenter/
webfsd -F -p 9876

