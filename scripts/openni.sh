#!/bin/bash

roslaunch openni_launch openni.launch depth_registration:=true 
exec $SHELL
