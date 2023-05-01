# remote_actuator

[![License](./apache20.svg)](./LICENSE.txt)

This is a ROS2 package providing the base class for all actuator drivers
and the interface for all modules looking to interact with actuators.

## Ros2_control and hardware_interface

Most actuator drivers need to be integrated with [ros2_control](https://github.com/ros-controls/ros2_control) using [hardware_interface](https://github.com/ros-controls/ros2_control/tree/master/hardware_interface).

This package is used by
[remote_hardware_interface](https://github.com/openvmp/remote_hardware_interface) for position or velocity control.
So all drivers using `remote_actuator` as the base class become integrated
with [ros2_control](https://github.com/ros-controls/ros2_control) automatically.

## Design

The actuator drivers implement the base class provided by this package.
For C++ drivers, that means that they use
`remote_actuator::Implementation` as a parent class.
Its constructor requires an instance of `rclcpp::Node` to read the
parameters from.

<i>
Note:
There can be more than one instance of `remote_actuator::Implementation`
per node. However they will all be controlled by the same parameters.
Though the default value of the parameter `actuator_prefix` can be passed
into each individual constructor which is sufficient for common use cases.
</i>
<br/>
<br/>

The following methods need to be implemented by each driver:

```c++
public:
  /* has_position must return true if it supports position control */
  virtual bool has_position() override;
  /* has_velocity must return true if it supports velocity control */
  virtual bool has_velocity() override;
protected:
  /* position_set_real_ is called to control the position */
  virtual void position_set_real_(double) override;
  /* velocity_set_real_ is called to control the velocity */
  virtual void velocity_set_real_(double) override;
```

The users can chose one of three ways to interact with child classes of `remote_actuator::Implementation`:

- Link with the driver directly to make native API calls:

  ```c++
  auto actuator = \
    std::make_shared<driver_package::DriverClass>(node, "/motor1");
  actuator->set_position(0.0);
  actuator->set_veloicty(1.0);
  ```

  In this case the driver is running within the same process and it is
  destroyed when the `actuator` object is destroyed.

- Link with `remote_actuator` to make ROS2 interface (DDS) calls
  (locally or over a network):

  ```c++
  auto actuator = \
    std::make_shared<remote_actuator::RemoteInterface>(node, "/motor1");
  actuator->set_position(0.0);
  actuator->set_veloicty(1.0);
  ```

  In this case the driver can be elsewhere within DDS' reach
  (same process or another side of the globe).

- Let the runtime make the choice between the above two options:

  ```c++
  auto actuator = \
    remote_actuator::Factory::New(node, "/motor1");
  // or
  auto actuator = \
    driver_package::Factory::New(node, "/motor1");
  ```

  In this case the boolean value of the parameter `actuator_is_remote`
  determines whether the driver is instantiated locally or if a remote
  interface is used to reach the driver instantiated elsewhere.
  Please, note, the trivial class `driver_package::Factory`
  (similar to `remote_actuator::Factory`) has to be written
  to support this use case.

No matter which option is chosen,
the driver will expose the following ROS2 interfaces.

### Parameters

- `actuator_prefix`: the prefix to ROS2 interfaces exposed by this driver
- Minimum and maximum values for filtering input as well as to support logic in other related modules (e.g. for [PWM control](https://github.com/openvmp/actuator_pwm)):
  - `actuator_position_min` and `actuator_position_max` for position controlled actuators
  - `actuator_velocity_min` and `actuator_velocity_max` for position controlled actuators
- `actuator_is_remote`: instructs the factory class (if any) whether
  to instantiate the driver locally or to connect to a remote instance

### Topics

#### Subscribers

- `<namespace>/<actuator_prefix>/set_position`: set the position
- `<namespace>/<actuator_prefix>/set_velocity`: set the velocity

#### Publishers

- `<namespace>/<actuator_prefix>/position`: the last position
- `<namespace>/<actuator_prefix>/velocity`: the last velocity

#### Services

- `<namespace>/<actuator_prefix>/set_position`: set the position
- `<namespace>/<actuator_prefix>/set_velocity`: set the velocity

## Examples

### Em2rs stepper driver series

See [em2rs_driver](https://github.com/openvmp/stepper_driver_em2rs) for an example of a driver that implements
`remote_actuator`.
