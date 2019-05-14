# Welcome to Marquette's NASA RMC Team Page

## The Project

The project was to build an autonomous mining robot to enter into the NASA Robot Mining Competition which was held in May 2019. Overall the project was fairly successful. While the robot had the capabilities to be autonomous, in the end there were too many technical hurdles to overcome before the final competition run in 2019. [Video of Run](https://youtu.be/aWYQTzyjIGM?t=6405)

## Robot Description and Components

The robot uses treads from [Super Droid Robots](http://superdroidrobots.com). It has a digging arm supported by two sets of linear actuators from from Midwest Motion and driven by two [Roboclaw motor drivers](https://www.pololu.com/product/3284). The digging bucket scoop belt is made from leather and 3D printed PLA infused with carbon fiber for the buckets. The dumping belt uses a vinyl belt reinforced with webbing and rubber rungs from Automation Direct. The robot runs on two 4Wh 4 cell LiPo battery packs in series from Turnigy with a nominal voltage of about 36V. This was regulated to 24V for the drives and 12V for the control boards using some home made regulator circuits. The main control board is the UDOO x86 Adv+ running Ubuntu 18.04. The locomotion, digging, and dumping were all operated using Anaheim Automation [BLWRPG17](https://www.anaheimautomation.com/products/brushless/brushless-gearmotor-item.php?sID=157&pt=i&tID=98&cID=47) Brushless DC motors with planetary gearbox and driven with the [ODrive Motor Controller](https://odriverobotics.com/) which we loved to work with. These motors we found to not have quite enough torque for this application but were very durable, reliable, and easy to work with. The robot used the Kinect v1 for XBox 360 as a camera as it also provides distance point cloud data. The other sensors we used were current and voltage sensors from [Phidgets](https://phidgets.com) and their 3/3/3 high presision IMU. We were impressed with all of phidgets products and how easy they were to work with.

## Thank Yous
We are so thankful to the University of Alabama for hosting the competition and all the volunteers who helped put on this amazing competition.

We would also like to thank Marquette University, Collins Aerospace, Komatsu, WSGC, Real Chili, ODrive, Phidgets, and the many family members and friends who gave us support and assistance for this project.

## The Team

1. Matthew Braccio - Project Lead
2. Erik LeBeau - Mechanical Design Lead
3. Zach Pederson - Mechanical Design / Treasurer
4. Tim Campbell -  Manufacturing
5. Dean Koumoutsos - Electrical
6. Zach Nordgren - Controls
6. Nathan Faust - Automation / Secretary

