# snakelib

A set of ROS packages for simulating and controlling Biorobotics Lab snake robots. More detailed documentation is available under the docs folder.

## User manual
Available at this [google doc](https://docs.google.com/document/d/1_hyYKsMlmH89wEYLgbvUxjnrGfiJdQpdpSmHiED1VWU/edit#heading=h.n5sfg8qwljtv).

## Supported snakes:
* ReU Snake
* SEA Snake
* R Snake

## Target platforms:
* ROS Noetic

## Notes on contributing:

Please follow [this convention](http://docs.ros.org/en/kinetic/api/catkin/html/howto/format2/installing_python.html) when organizing Python modules. The key points to follow are:
* Place Python modules under the ```src/your_package``` subdirectory, making the top-level module name the same as your package.
* All executable Python programs go in a subdirectory names ```nodes/``` or ```scripts/```, where nodes are ROS nodes and scripts are other executable Python scripts.

Follow the [ROS 1.0 PyStyle Guide](http://wiki.ros.org/PyStyleGuide).

Use of [Black](https://pypi.org/project/black/) is recommended.

For commenting, follow the [Google styleguide](https://github.com/google/styleguide/blob/gh-pages/pyguide.md#38-comments-and-docstrings).
