#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/src/
rsync -avzh ./include/sonar_positioning/*.hpp 	eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/include/sonar_positioning/
rsync -avzh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/
rsync -avzh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/

rsync -avzh ./launch/*.launch	 				eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/launch/
rsync -avzh ./srv/*.srv	 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/srv/
rsync -avzh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/
#rsync -avzh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/
#rsync -avzh ./urdf/*.urdf						eurathlon_vm:/home/euratlhon/uwesub_msc/src/sonar_positioning/urdf/

echo "All done, Good Success!"