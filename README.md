# Software for Couchmobile

## Setup instructions
### Prerequisites
1. Have Ubuntu operating system installed (preferably natively rather than in a virtual machine due to speed and memory usage)
2. Have ROS melodic installed on your machine. [Installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. Have configured your basic ROS melodic installation. [Instructions here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

### Instructions
1. Create a ROS workspace using
```bash
mkdir -p ~/couchmobile/src/         # Creates the workspace folder
cd ~/couchmobile/                   # Goto the root of workspace folder
catkin_make                         # Initialize workspace
```
2. Clone this repository into the src/ folder of your workspace
```bash
cd ~/couchmobile/src/
git clone https://github.com/CouchmobileUNSW/couchmobile-software.git
```
3. Run the install scripts for couchmobile. Ensure that your computer has enough RAM and space for compilation and installation of all dependencies. 8GB of RAM is recommended for this step, otherwise you will need to run these steps manually.
```bash
cd ~/couchmobile/src/couchmobile-software/scripts/
./00-install_all.sh
```

