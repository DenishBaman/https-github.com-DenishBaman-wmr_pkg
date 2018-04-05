# Wheel Mobile Robot(WMR) package
A framwork to make over the network commincation setup between unmmaned ground vehicle and ground station. 

*Download repository 
```
cd
mkdir -p ~/wmr_pkg/src
cd ~/wmr_pkg/
catkin_make

cd ~/Download && git clone https://github.com/DenishBaman/wmr_pkg.git
sudo cp -r wmr_pkg/* ~/wmr_pkg/src/.
cd ~/wmr_pkg

```


# State Feedback

A package for sensor data acquisition and utilization for vehicle state feedback.

1. GPS 

Acquire the raw GPS data and determine the local position of the vehicle.  
