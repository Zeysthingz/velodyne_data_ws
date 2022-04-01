### Lidar Data Segmentation as Ground and Non-Ground

In this repo, the point cloud data from lidar is separated as ground and non-ground then visualized on rviz.

####  <span style="color: pink">Installation</span>
* ROS
* Ubuntu 20.04 
* PCL 1.12.0

To download PCL from source :

    wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz | tar xz  ---> dependency

    cd eigen-3.3.7 && mkdir build && cd build

    sudo make install

    cd ../.. && rm -rf eigen-3.3.7/ && rm -f eigen-3.3.7.tar.gz

    cd ~/programs

    git clone https://github.com/PointCloudLibrary/pcl.git --> download source file where you want to locate.

    cd pcl
    
    mkdir build
    
    cd build
    
    cmake -D BUILD_CUDA=0 -D BUILD_GPU=0 -D WITH_CUDA=0 -D CMAKE_INSTALL_PREFIX=~/libraries/libpcl-1.11.1 -D CMAKE_BUILD_TYPE=Release ..
    
    make -j 20
    
    sudo make install

####  <span style="color: pink">How to add Cmake file</span>
    set(PCL_DIR "~/libraries/libpcl-1.12.1/share/pcl-1.12/")
    find_package(PCL.1.12.1 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS}) 

I created new publisher for ground point cloud node and worked on a rosbag file.

    rosbag play -l rosbag_file_name
    rosrun udacity-pcl-tut udacity-pcl-tut_exec 

You can see the ground non-ground segmentation results from the link below on rviz.

https://www.youtube.com/watch?v=4L6A2cyH_II&list=PLI9niWoEI8BghYVuZ1c0jHLidw4vRO84V
* orange --> ground point cloud 
* blue --> non-ground point point cloud

<span style="color: orange">You can use .rviz saved configuration file to open point clouds on rviz.</span>


#### Why Do We Have Outliers ?

As you can see below the Ransac algorithm assumes inliers the greatest plain as groundand tries draw a line on it  , if the ground is curved rather than plain, the algorithm fails to make an exact distinction.

![alt text](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/random_sample_example2.png)

You can increase iteration and change distance parameter between points to get better results.