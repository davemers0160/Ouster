# Ouster Project
This project is designed to control and collect data from the Ouster OS-1-64 Lidar system (https://www.ouster.io/).  The code here is a pseudo fork from the ouster github location (https://github.com/ouster-lidar/ouster_example).

However, the repo is not directly forked from theirs so it does not keep up with changes automatically.  The Ouster repo is solely based on development on Ubuntu and the ROS operating system.  These requirements make it it not very friendly for using in Windows.  

This repo tries to solve that problem and make it easier to use without the ROS dependencies.

## Compiling the project for Windows
The project build build is based on the CMake build system.  To begin create a directory called build.  Open the CMake GUI and select the location for the CMakeLists.txt file and then select the locations for where to place the build files.

Press the Configure button and select the 64-bit compiler.  Press OK and wait until it is complete.  Next make sure the advanced check box (located in the upper right) is checked and then look for the "USE_AVX_INSTRUCTIONS" item and check the box.  Select the Configure button again and then press the Generate button to create the project files.

Once the project is generated press the Open Project button to open the project up and start compiling it.

## Running the Executable
To run the code there are several default parameters that will be used.  To change the defaults a file can be specified.  An example file (lidar_configfile.txt) is supplied in the repo that explains what options to change.  To include the file simply pass it in as the first parameter to the executable.


# Dependencies
This project depends on a secondary repo matained here: https://github.com/davemers0160/Common

This repo contains code that is common across several projects (hence the name :smiley:).  The CmakeLists.txt file must be edited to point to the location of the Common repo.



