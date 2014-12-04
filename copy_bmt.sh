#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						bmt:/home/devel/catkin_ws/src/sonar_positioning/src/
rsync -avzh ./include/sonar_positioning/*.hpp 	bmt:/home/devel/catkin_ws/src/sonar_positioning/include/sonar_positioning/
rsync -avzh CMakeLists.txt 						bmt:/home/devel/catkin_ws/src/sonar_positioning/
rsync -avzh *.xml 								bmt:/home/devel/catkin_ws/src/sonar_positioning/

rsync -avzh *.launch	 						bmt:/home/devel/catkin_ws/src/sonar_positioning/
rsync -avzh *.yaml		 						bmt:/home/devel/catkin_ws/src/sonar_positioning/
rsync -avzh *.md								bmt:/home/devel/catkin_ws/src/sonar_positioning/
rsync -avzh ./urdf/*.urdf						bmt:/home/devel/catkin_ws/src/sonar_positioning/urdf/

echo "All done, Good Success!"