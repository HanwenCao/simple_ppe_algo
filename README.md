# simple ppe algo

This simple project is designed for pose estimation of markers / single marker.
only aruco & charuco families are supported right now.

## Dependencies
1. <https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md>  
2. OpenCV


## 1. Build and install
1. clone  
2. cd to simple_ppe_algo  
3. `mkdir build  && cd build`  
4. `cmake ..`  
5. `make`  
Then the following will be create.  
shared library: build/auto_market_ppe/libauto_market_ppe.so  
executable file: build/auto_market_ppe/testHandeyeTarget  
To use the shared library, clone the generated shared library and header files to your project, and you could refer to the test file in folder test/, for the reference of usage in detail.



## 2. Print marker
Go to <https://chev.me/arucogen/>  
Set `Dictionary = 4x4` and choose a suitable marker ID and a suitable marker size. Remember this size for futher use.  
Directly `Ctrl + P` to print your marker.




## 3. Test file workflow
1. Configurations  
    - Configuration of markers: The configurations of markers can be found in file include/handeye_target/handeyeTargetConfig.hpp. Note that some are overwritten in cpp. So leave hpp alone for now.
    - Configuration of camera: Camera's intrinsic params will be automatically detected if connected to realsense camera.



2. Test with data  
Make sure in `handeyeTargetTest.cpp`, the following are modified according to the printed marker.
```
charuco_cfg.marker_measured_size = 0.09f; //黑色方块尺寸，single模式下没用
charuco_cfg.marker_measured_separation = 0.08f; //每个二维码尺寸
```


run `./testHandeyeTarget`  
An image be created as chAruco_5X5_DICT4X4.jpg for printing and visualization.
First it will load an image named `img_0.jpg`.
And a video stream will start if connected to realsense camera.  
In this mode youcan:  
**Press "s" to switch between single marker / chessboard detection mode.**  
**Press "q" to quit.**  
**Press "u" to add current marker id you want to detect, this only works in single marker detection mode.**  
**Press "d" to reduce current marker id you want to detect, this only works in single marker detection mode.**  

## 4. Trouble shooting

