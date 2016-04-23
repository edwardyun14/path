#### PLEASE USE visgraph.py, NOT path.py

#### Installation
Install libgeos-dev if you don't have it (native geo library)  
`sudo apt-get install libgeos-dev`  
Also install shapely (Python wrapper providing geometric functions)  
`python -m pip install shapely`  
or whatever python package manager you use

Depending on your python installation, you might need to install yaml as well.  

#### Usage
To use the ROS pacakge, start roscore, and then run publisher.py in python 2.7, passing the file of the obstacle map in as a parameter
Ex: `python publisher.py my_map.json`

