echo Moving custom ompl files
sudo mv /usr/local/include/ompl ~/Desktop/custom_ompl/
sudo mv /usr/local/lib/libompl* ~/Desktop/custom_ompl/
echo Delete ROS_NOBUILD
rm ROS_NOBUILD
echo Compiling...
make clean
rosmake
echo Create ROS_NOBUILD
touch ROS_NOBUILD
echo Move back custom ompl files
sudo mv ~/Desktop/custom_ompl/libompl* /usr/local/lib/
sudo mv ~/Desktop/custom_ompl/ompl /usr/local/include/

