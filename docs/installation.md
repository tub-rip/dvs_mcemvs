## Dependencies

### Install ROS Noetic (Ubuntu 20.04)

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

The code has been tested on ROS Noetic. Depending on the ROS distribution you installed, you might have to use `kinetic` or `melodic` instead of `noetic` in the previous command.

### Add packages to the catkin workspace

**Clone** this repository into the `src` folder of your catkin workspace.

	cd catkin_ws/src
	git clone git@github.com:tub-rip/dvs_mcemvs.git

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))
- [Google Logging Library (glog)](https://github.com/catkin/catkin_simple.git)
- [Gflags (formerly Google Commandline Flags)](https://github.com/ethz-asl/gflags_catkin)
- [minkindr](https://github.com/ethz-asl/minkindr) (for dealing with poses). minkindr depends on [eigen_catkin](https://github.com/ethz-asl/eigen_catkin.git) and [eigen_checks](https://github.com/ethz-asl/eigen_checks.git), which are therefore also required.
- [vicon](https://github.com/KumarRobotics/vicon) for reading poses in the form of vicon_msg

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < dvs_mcemvs/dependencies.yaml

The previous command should clone the repositories into folders *catkin_simple*, *rpg_dvs_ros*, etc. inside the src/ folder of your catkin workspace, at the same level as this repository *dvs_mcemvs*. They should NOT be inside the *dvs_mcemvs* folder.

Additional ROS tools needed (specified in the [package.xml](package.xml) file):

	sudo apt-get install ros-noetic-image-geometry
	sudo apt-get install ros-noetic-tf-conversions

## Compiling

**Compile this package**:

	catkin build mapper_emvs_stereo
	
After building, at least the first time, remember to run:

	source ~/catkin_ws/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/mapper_emvs_stereo/

An alternative command to start from scratch (cleaning all catkin packages) is (to be used with *caution*): `catkin clean`
