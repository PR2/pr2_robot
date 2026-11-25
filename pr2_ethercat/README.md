(formerly [pr2_etherCAT](pr2_etherCAT) in Groovy and earlier)

## Nodes

### pr2_ethercat

`pr2_ethercat` is the node that executes the main loop for controlling
the EtherCAT devices (including all motors) on the PR2. It loads
[pr2_controller_manager](https://index.ros.org/p/pr2_controller_manager),
which in turns loads and runs controllers that act directly on the
robot's motors. The ROS API detailed below is the interface for
controlling `pr2_ethercat` itself; as a result of loading the
controllers, the `pr2_ethercat` process offers a broader, and varying
ROS API. Superuser privileges are required to run `pr2_ethercat`, to
allow it to set its scheduling priority to run as a real-time process.

#### Services

- `pr2_ethercat/reset_motors`
  ([std_srvs/srv/Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)):
  Reset the motors. This is required after a runstop is reset (by
  turning the red push-button on the back of the robot, or pressing the
  "Start" button on the wireless runstop), and after the motors have
  been halted via the `halt_motors` service. Most often called from
  [pr2_dashboard](pr2_dashboard).
- `pr2_ethercat/halt_motors`
  ([std_srvs/srv/Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)):
  Halt the motors. To reset them, call `reset_motors`. Most often
  called from [pr2_dashboard](pr2_dashboard).

#### Permissions

Running `pr2_ethercat` requires the following "capabilities":

- **CAP_IPC_LOCK** Lock memory (mlockall())
- **CAP_SYS_NICE** Set real-time scheduling policies
- **CAP_NET_RAW** Use raw sockets (for EtherCAT communications)
  Superuser has these capabilities, but they can also be granted to
  another executable by setting file system attributes on the executable
  file. The `pr2-grant` program is a setuid-root program which copies
  `pr2_ethercat` to a part of the filesystem where attributes can be
  set (`/var/tmp`), and then grants the necessary capabilities to be
  able to run `pr2_ethercaat`.

#### Usage



```
Usage: ./pr2_ethercat [options]
  Available options
    -i, --interface <interface> Connect to EtherCAT devices on this interface
    -x, --xml <file|param>      Load the robot description from this file or parameter name
    -r, <param>                 Load the robot description from this parameter
    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices
    -h, --help                  Print this message and exit
```



#### Example

`pr2_ethercat` is usually executed from a launch file, such as
[pr2_bringup](https://index.ros.org/p/pr2_bringup)/pr2.launch. Common
configuration: 

```
<node name="realtime_loop" machine="c1" launch-prefix="pr2-grant" pkg="pr2_ethercat" type="pr2_ethercat" args="-i ecat0 -x robot_description"/>
```



### pr2-grant

`pr2-grant` is a utility that allows a regular user to run
`pr2_ethercat` with appropriate capabilities. See the section in
`pr2_ethercat` on "Permissions."

## reset_motors.py

`reset_motors.py` is a simple command-line script that calls the
`pr2_ethercat/reset_motors` service.



```
$ rosrun pr2_ethercat reset_motors.py
```
