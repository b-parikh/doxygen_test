# DOXYGEN_TEST

---

This package shows how to document ROS packages with Python and C++ code.

Within this package, there are two entities modeled as ROS nodes:
1. creator
2. requester

The creator is tasked with creating regular polygons (polygons with equal side lengths). After 
creating them, the creator stores these polygons. The requester asks the creator to create
polygons with certain specifications. At a set frequency, the creator will publish a list of all
the polygons that it has created so far. The requester reads this list and prints it out on
ROS_INFO. 

To generate documentation, install Doxygen 
