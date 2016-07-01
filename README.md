# Presentation

This project is splitted into two big part. One part permit to realize a tracking from a velodyne device and the other part permit to analyse the behavior of the tracked drivers.

In all parts, the used datasets are provided by [KITTI](http://www.cvlibs.net/datasets/kitti/) ([synced+rectified data]).

# Installation

The code was tested using : 


* ROS Indigo, Libosmium library (2.6.1), PLplot library (5.11.1), dlib library (18.18), KFilter library (1.3) and glm on Ubuntu 14.04 LTS.
* ROS Kinetic, Libosmium library (2.7.2), PLplot library (5.11.1), dlib library (19.0), KFilter library (1.3) and glm on Ubuntu 16.04 LTS.

## Libosmium library

[Libosmium](http://osmcode.org/libosmium/) is a fast and flexible C++ library for working with OpenStreetMap data. Also, Osmium is a header-only library, so there is nothing to build for the library itself.

To use it, you must install his important and extra [dependencies](https://github.com/osmcode/libosmium/wiki/Libosmium-dependencies) and put his include folder into /include/osm/libosmium folder.

## PLplot library

[PLplot](http://plplot.sourceforge.net) is a cross-platform software package for creating scientific plots. Here, the PLplot core library is used to create standard x-y plots.

To install this library, download or clone the package presents into the downloads section and follow the installation readme.

During the installation process, if you meet a problem with qt, qt which does not use into this project, you can fix that by a simply way. Go to plplot package folder. Then open /cmake/modules/drivers-init.cmake and change "OFF" by "ON" at this line : 

    option(DEFAULT_NO_QT_DEVICES
      "Disable all qt devices by default (ON) or enable qt devices individually by default (OFF)"
      ON
      ) 

## dlib library

[Dlib](http://dlib.net) is a C++ toolkit containing machine learning algorithms and tools for creating complex software in C++ to solve real world problems. Here, only Support Vector Machines (SVM) methods are used.

To use it, download the sources from dlib website. Then, go into package folder and build the library. To do that, unfollow the installation process given by dlib and type these lines in the terminal :
    
    mkdir build
    cd build
    cmake ..
    make
    make install

## KFilter library

The code uses the [KFilter library](http://kalman.sourceforge.net/) for the kalman filter implementation.

The CMakeLists.txt file assumes that the KFilter is installed in
the default directory (/usr/local/lib).

To install this library, download the sources and follow "install.txt". Then go to /usr/local/lib and type :
    ranlib libkalman.a

## glm

[OpenGL Mathematics (GLM)](http://glm.g-truc.net/0.9.7/index.html) is a header only C++ mathematics library. To obtain it, enter this line in a terminal :
    
    sudo apt-get install libglm-dev  

# How to run

The code assumes that the KITTI datasets are stored in the datasets folder
maintaining the KITTI folder structure.

For example:

    /datasets/2011_09_26/2011_09_26_drive_0005_sync/.

## Tracking

### Input

Please see parameters.yaml.

### Run

The code is split into two steps with two launch files:

    kitti2don.launch
    velodyne_tracking.launch

Both these launch files need the input arguments **"date"** and **"id"** to specify the path.
For example to run the above dataset.

    roslaunch velodyne_tracking kitti2don.launch date:=2011_09_26 id:=5
    roslaunch velodyne_tracking velodyne_tracking.launch date:=2011_09_26 id:=5

Additionally the argument **"rviz:=false"** can be added if you don't want to visualize the process in RVIZ. Also, if rviz is enable it will be possible to see the points cloud and the Bbox into a grayscale camera which takes place on the top of the car. To disable these options you can use : **"img:=false"**, **"bb:=false"** and **"point:=false"**.

### Output

The extracted paths are then saved to a csv file with the name **"date-id.csv"**.
With the example above it would be:

    /datasets/2011_09_26/2011_09_26_drive_0005_sync/path/2011_09_26-0005.csv

This default position can be changed by modifying the parameter "save_path"
in the file parameters.yaml.

### Output : file Structure

For each dataset a csv file is saved containing all the extracted paths, it is formatted as follows:

    # oxts origin
    "Line containing the first oxts measurements for that dataset, copy of /oxts/data/0000000000.txt."

    # Track: 1
    timestamp1, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz
    timestamp2, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz
        .
        .
    timestampN, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz

    # Track: 2
    timestamp1, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz
    timestamp2, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz
        .
        .
    timestampN, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz

where each value is defined as:

    timestamp      : Time the position occurred, "year-month-day H:mm:ss.ms"
    x              : the x position with oxts position as origin.
    y              : the y position with oxts position as origin.
    z              : the z position with oxts position as origin.
    vx             : the velocity in x direction.
    vy             : the velocity in y direction.
    vz             : the velocity in z direction.
    qx, qy, qz, qw : quaternion with specifying the rotation of the oriented bounding box.
    sx             : width of the x direction of the oriented bounding box before translation.
    sy             : width of the y direction of the oriented bounding box before translation.
    sz             : width of the z direction of the oriented bounding box before translation.

Here the bounding box is defined as the side lengths (sx/sy/sz) of the box aligned with the x/y/z axis.

The position (x/y/z) and the quaternion (qx/qy/qz/qw)  can then be combined into a transformation matrix to
get the correct position.

## Driver Behavior

### Input

Please see driverBehaviorParameters.yaml

### Run

