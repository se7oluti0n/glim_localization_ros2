# glim_ros2


## **Change log **
This version of GLIM is based on v1.0.4, so it's depend on gtsam_points v1.0.4
- **Localization** module: a modification from **global_mapping** module, load saved map and to the matching with saved map 
- **Wheel Odometry Constraint**: use wheel odometry data to handle Lidar odometry drift 
- **Localization GUI**: a modification from offline-viewer to add load map and click-to-relocalize function. Also include cropping tool for 2D Costmap export. ]
[Youtube Demo with Localization GUI](https://www.youtube.com/watch?v=qgoBgG9_V48)
- **Octomap**: export to 2D top-down view for robot Navigation
- **GPS Factor**: Work with good GPS, but you may want to try Koide san GNSS extension first

## **HOW TO USE**
-   Check out launch file [demo_localization.launch.py](./launch/demo_localization.launch.py)
-  First build a map in `mapping` mode, (let ros param `localization` is `false`)
-  Then use the map for localization 
-  Check out the video https://www.youtube.com/watch?v=qgoBgG9_V48
-  Examples dataset: Checkout the Liosam datasets (**walking** or **campus**): https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq. And use the config file in glim config folder [config/liosam]
(https://github.com/se7oluti0n/glim_localization/tree/localization/config/liosam)
