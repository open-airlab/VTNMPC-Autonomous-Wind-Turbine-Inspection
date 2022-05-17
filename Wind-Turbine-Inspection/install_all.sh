# Fetching the catkin submodule
echo "========================================="
echo "Updating all submodules"
echo "========================================="
git submodule update --quiet --init --recursive --remote --jobs 4 || true
git submodule sync --recursive
git submodule update --init --recursive --remote --jobs 4


# PX4
echo "========================================="
echo "PX4 setup"
echo "========================================="
sleep 1.5
cd WTI_px4/
#./home/jonas/code/nodes/WTI_px4/install_geographiclib_datasets.sh
echo "installing: geographiclib_datasets.sh"
sleep 1
sudo chmod -x ./install_geographiclib_datasets.sh

cd ../

echo "========================================="
echo "AIRSIM setup"
echo "========================================="
cd airsim/
echo "running: setup.sh"
sleep 1.5
./setup.sh
echo "========================================="
echo "AIRSIM build"
echo "========================================="
echo "running: build.sh"
sleep 1.5
./build.sh

echo "========================================="
echo "AIRSIM ROS"
echo "========================================="
cd ros/
echo "running: catkin build"
sleep 1.5
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
cd ../../




