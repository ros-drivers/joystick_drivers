^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spacenav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.0 (2023-10-31)
------------------

3.2.0 (2023-10-10)
------------------
* add option to use TwistStamped (`#251 <https://github.com/ros-drivers/joystick_drivers/issues/251>`_)
* Changed name of executable in launch files to match installed node (`#230 <https://github.com/ros-drivers/joystick_drivers/issues/230>`_)
* Fix publishing of spacenav button values (`#243 <https://github.com/ros-drivers/joystick_drivers/issues/243>`_)
* Fix from-source build with missing dependencies (`#242 <https://github.com/ros-drivers/joystick_drivers/issues/242>`_)
* Installing libspacenav.so to lib/ for spacenav_node execution via ros2 run (`#229 <https://github.com/ros-drivers/joystick_drivers/issues/229>`_)
* Contributors: Borong Yuan, Stefan Scherzinger, chriseichmann

3.1.0 (2022-01-28)
------------------
* Install includes to include/ and misc CMake fixes (`#225 <https://github.com/ros-drivers/joystick_drivers/issues/225>`_)
* Style fixes for newer cpplint.
* Contributors: Chris Lalancette, Shane Loretz

3.0.1 (2022-01-28)
------------------
* Make sure to ament_target_dependencies on rclcpp_components (`#209 <https://github.com/ros-drivers/joystick_drivers/issues/209>`_)
* Contributors: Chris Lalancette

3.0.0 (2021-03-12)
------------------
* spacenav node changed for ros2 (`#194 <https://github.com/ros-drivers/joystick_drivers/issues/194>`_)
* Contributors: Nils Schulte

1.13.0 (2019-06-24)
-------------------

1.12.0 (2018-06-11)
-------------------
* Adding tested spacenav scaling
* Added README
* Addressed numerous outstanding PRs.
* Changed package xml to format 2
* Reduce the number of scale params in spacenav_node
* Add a scale parameter to each axis in spacenav_node
  Do not apply the scale to the joy topic.
* Contributors: Gaël Ecorchard, Jonathan Bohren, jprod123

1.11.0 (2017-02-10)
-------------------
* Update dependencies to remove warnings
* Contributors: Mark D Horn

1.10.1 (2015-05-24)
-------------------
* Add full_scale parameter and apply to offset
* Remove stray architechture_independent flags
* Contributors: Gaël Ecorchard, Jonathan Bohren, Scott K Logan

1.10.0 (2014-06-26)
-------------------
* First indigo release
