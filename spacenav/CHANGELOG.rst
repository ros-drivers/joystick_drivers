^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spacenav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix publishing of spacenav button values (`#243 <https://github.com/ros-drivers/joystick_drivers/issues/243>`_)
  * Fix publishing of spacenav button values
  We now always publish the two default button states, in which
  0 = not pressed
  1 = pressed.
  Additional buttons emerge and stay in that list upon being pressed for
  the first time.  This supports more advanced spacenav devices with a
  yet unknown number of buttons.
  Fixes `#223 <https://github.com/ros-drivers/joystick_drivers/issues/223>`_
  * Fix format
  * Switch back to shared memory transport
  We now only keep the buttons' memory as class member.
* Fix from-source build with missing dependencies (`#242 <https://github.com/ros-drivers/joystick_drivers/issues/242>`_)
  Using a variable for dependencies is cleaner for linking and
  export.
* Installing libspacenav.so to lib/ for spacenav_node execution via ros2 run (`#229 <https://github.com/ros-drivers/joystick_drivers/issues/229>`_)
  Fixed `#221 <https://github.com/ros-drivers/joystick_drivers/issues/221>`_
* Contributors: Stefan Scherzinger, chriseichmann

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
