# autoware_core package
core package for Autoware.  
this package provides http based API for autoware.  
autoware_core does not depands on ROS.
## How to use
roslaunch autoware_core autoware_core.launch

## API
### roslaunch API
path : /roslaunch  
You can send post command to this path.
#### request
```
{
    "package" : "diag_lib",
    "launch_filename" : "watchdog_sample.launch"
}
```
#### response
```
{
    "package" : "diag_lib",
    "launch_filename" : "watchdog_sample.launch",
    "response" : 
    {
        "success" : true,
        "description" : "hogehoge"
    }
}
```