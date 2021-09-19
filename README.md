# simple ppe algo

This simple project is designed for pose estimation of markers / single marker.
only aruco & charuco families are supported right now.



## 1. Build and install
1.clone the repo and source ros2-foxy .  
2.run colcon build from the root path of this repo.  
shared library: build/auto_market_ppe/libauto_market_ppe.so  
test file: build/auto_market_ppe/testHandeyeTarget



## 2. Usage
Clone the generated shared library and header files to your project, and you could refer to the test file in folder tset/, for the reference of usage in detail. 
https://chev.me/arucogen/


## 3. Test file workflow
1. Configurations  
The configurations of markers can be found in file include/handeye_target/handeyeTargetConfig.hpp.  
And the configurations of camera's intrinsic params will be automatically done if connected to realsense camera.

   
2. Marker generation  
An image be created as chAruco_5X5_DICT4X4.jpg for printing and visualization.

3. Test with data  
First it will load an image named img_0.jpg for detection test.
And the test with video stream will start if connected to realsense camera.  in this mode youcan:  
**Press "q" to quit.**  
**Press "s" to switch between single marker / chessboard detection mode.**  
**Press "u" to add current marker id you want to detect, this only works in single marker detection mode.**  
**Press "d" to reduce current marker id you want to detect, this only works in single marker detection mode.**

## 4. Trouble shooting

