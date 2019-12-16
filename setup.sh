# Use this script to install rgbdslam on ROS kinetic (simple do "$ bash setup.sh" from the same directory in terminal)
# Make sure to modify the following appropriately:

#### G2O_REPO_DIR : The place where you want to make your g2o library
#### CATKIN_WORKSPACE : The location of your caktin workspace

#If you have a fast machine with 4GB RAM or more, increase the 
#two occurences of "-j2" below to parallelize the compilation more

#Prepare System
if test ! -d /opt/ros/kinetic; then
  echo This script assumes ROS kinetic to be installed
  echo The directory /opt/ros/kinetic was not found
  exit 1
fi

# Choose a location to make your g2o (not in your catkin_WS)
G2O_REPO_DIR=/home/mscv/src
echo "This script puts g2o into '$G2O_REPO_DIR'. Edit this script to change the location."
echo "Press enter to continue, Ctrl-C to cancel"
read
mkdir -p $G2O_REPO_DIR

source /opt/ros/kinetic/setup.bash

echo
echo "Removing packages known to conflict (password required for apt-get)"
echo
sudo apt-get purge ros-kinetic-libg2o libqglviewer-dev

echo
echo "Updating ROS dependency database"
echo
rosdep update

echo "Install dependences for g2o"
sudo apt-get install libsuitesparse-dev libeigen3-dev
echo

echo
echo "Downloading, building and installing g2o"
echo
G2O_REPO_DIR1=$G2O_REPO_DIR/g2ofork
git clone -b c++03 https://github.com/felixendres/g2o.git $G2O_REPO_DIR1
mkdir $G2O_REPO_DIR1/build
cd $G2O_REPO_DIR1/build
cmake .. -DCMAKE_INSTALL_PREFIX=$G2O_REPO_DIR1/install -DG2O_BUILD_EXAMPLES=OFF
nice make -j2 install

echo
echo "Preparing catkin workspace for rgbdslam_v2"
echo
# Put the path of your catkin workspace
CATKIN_WORKSPACE=/home/mscv/ros/kinetic/catkin_ws
mkdir -p $CATKIN_WORKSPACE/src
cd $CATKIN_WORKSPACE/src

echo
echo "Downloading rgbdslam_v2"
echo
#Get and build rgbdslam_v2
export G2O_REPO_DIR=$G2O_REPO_DIR1/install
git clone -b kinetic https://github.com/felixendres/rgbdslam_v2.git $CATKIN_WORKSPACE/src

#Install missing dependencies
rosdep install rgbdslam
echo
echo "Building rgbdslam_v2"
echo
nice catkin_make -C $CATKIN_WORKSPACE -j2




