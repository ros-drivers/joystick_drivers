^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wiimote
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2023-10-10)
------------------
* Revert "Fix dependency issues." (`#259 <https://github.com/ros-drivers/joystick_drivers/issues/259>`_)
* Fix dependency issues.
* Contributors: Chris Lalancette, Joshua Whitley

3.1.0 (2022-01-28)
------------------
* Install includes to include/ and misc CMake fixes (`#225 <https://github.com/ros-drivers/joystick_drivers/issues/225>`_)
* Style fixes for newer cpplint.
* Contributors: Chris Lalancette, Shane Loretz

3.0.1 (2022-01-28)
------------------

3.0.0 (2021-03-12)
------------------
* Fix a warning while building wiimote_controller.cpp (`#201 <https://github.com/ros-drivers/joystick_drivers/issues/201>`_)
* fix compile error caused by missing include (`#197 <https://github.com/ros-drivers/joystick_drivers/issues/197>`_)
* Port over Wiimote to ROS2 Foxy (`#175 <https://github.com/ros-drivers/joystick_drivers/issues/175>`_)
* Contributors: Chris Lalancette, Kuni Natsuki, Kurt Wilson

1.13.0 (2019-06-24)
-------------------
* Merge pull request `#132 <https://github.com/ros-drivers/joystick_drivers/issues/132>`_ from mistoll/clean_exit
* return instead of shutdown
* Merge pull request `#128 <https://github.com/ros-drivers/joystick_drivers/issues/128>`_ from ros-drivers/fix/tab_errors
* Cleaning up Python indentation.
* Merge pull request `#125 <https://github.com/ros-drivers/joystick_drivers/issues/125>`_ from mistoll/check_connection
* Check if wiimote is still connected.
* Merge pull request `#123 <https://github.com/ros-drivers/joystick_drivers/issues/123>`_ from cclauss/modernize-python2-code
* Modernize Python 2 code to get ready for Python 3
* Merge branch 'master' into indigo-devel
* Contributors: Joshua Whitley, Matthew, Michael Stoll, cclauss

1.12.0 (2018-06-11)
-------------------
* Addressed numerous outstanding PRs.
* Fixed issue when nunchuk wasn't connected
* Update README.md
* Added testing proceedures for wiimote
* Resolved div-by-zero error race condition on startup.
* Added testing proceedures for wiimote
* Fixed issue when nunchuk wasn't connected
* Changed package xml to format 2
* Added wiimote testing script
* Add pairing params to wiimote_node
  * bluetooth_addr
  * pair_timeout
* Contributors: Jonathan Bohren, Matt Vollrath, jprod123, psimona

1.11.0 (2017-02-10)
-------------------
* Sample Teleop Implementation for Wiimote
* C++ Implementation of Wiimote Controller Node
* Add queue_size to remove ROS Warning
* Update dependencies to remove warnings
* Contributors: Mark D Horn

1.10.1 (2015-05-24)
-------------------

1.10.0 (2014-06-26)
-------------------
* First indigo release
