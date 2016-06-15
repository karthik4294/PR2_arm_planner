echo Moving groovy ompl files
sudo mv /opt/ros/groovy/include/ompl ~/Desktop/groovy_ompl/
sudo mv /opt/ros/groovy/lib/libompl.so* ~/Desktop/groovy_ompl/
echo Delete ROS_NOBUILD
rm ROS_NOBUILD
echo Compiling...
make clean
rosmake
echo Create ROS_NOBUILD
touch ROS_NOBUILD
echo Move back groovy ompl files
sudo mv ~/Desktop/groovy_ompl/libompl.so* /opt/ros/groovy/lib/
sudo mv ~/Desktop/groovy_ompl/ompl /opt/ros/groovy/include/

