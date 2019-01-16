# Mnist Digit Tracker

Write 2 ROS nodes in a single ROS package. 
************************************
# Installation
Dependencies: Ros melodic, Opencv, TensorFlow, Linux

To install, clone this repo into your ros workspace src/ directory

************************************
# How to Run

Before Starting, **source your devel/setup.bash** every time you open a new terminal
Then run catkin_make from your catkin_workspace directory

If catkin make fails because a failure to find opencv, follow the instructions inside the
CMakeLists.txt file that start where it says 
"#!!!!!!!!!!!!!!!!!!!!!!!!!!OPENCVERRORS!!!!!!!!!!!!!!!!!!!!!!!!!!#"


1) Start Roscore in one terminal
2) In a new terminal run: roslaunch mnist_digit_tracker rviz.launch
3) In a new terminal run: rosrun mnist_digit_tracker mnist_publisher_node
4) In a new terminal run: rosrun mnist_digit_tracker mnist_subscriber_node.py

You should be able to enter a digit into the terminal where you ran the publisher node, and see it be traced by the 2 link manipulator. 

To quit ctrl-c in the respective terminals.
There is a bug with the publisher node where after you ctrl-c, you need to press enter afterwards to quit the program.

This project is still missing autogenerated comments for sphinx, but will have them soon.
************************************
# Project Insights

#### In the Mnist Publisher (C++)
- With the main design choice that I encountered was in how to take in input from the keyboard in a non-blocking fashion
- I considered 2 methods: 
   1) using a separate rosnode that broadcasts the most recently pressed key
   2) doing an OS-specific hack to disable the stdin blocking
- I ended up doing the OS-specifc hack because at the time, it seemed more efficient, as opposed to using a separate node.
- This approach ended up working, but wasn't as clean because I have  a bug where I need to provide a keypress after I type ctrl-c (SIGINT). I think I may have still encountered this bug if I had created a separate ros keyboard node, but it is hard to say without having have tried. 

#### In the Mnist Subscriber (Python)
- Proessing the image and training a classifier to identify which digit it was was not a time consuming task, however, deciding how to display my robot movement ended up taking almost 2 weeks.
- I initially tried to use the ROS package moveit! to simulate my movement, however I was unsuccessful in figuring out how to integrate my own 2d kinematics solver with it, and it turns out using moveit! abstracted away many of the learning experiences meant to be had with this project.
- I was eventually directed to write my own inverse kinematics solver, and mimic the dynamics of my robot using a low pass filter. 
1) I initially had quite a bit of trouble understanding how a filter would help me mimic my dynamics, but after graphing the function, it was clear to me how it fit into my project.
************************************
# Time breakdown
- 1 week to understand basic publisher subscriber model, and inspect how ros actually works
- 1 week to write my publisher. My C++ was quite rusty, and trying to parse through the mnist images ended up becoming a time-consuming function to integrate into my project.
- 1 week to understand URDF, inverse kinematics, and further understand how to use different messages and nodes in my project, as well as the time that I spent trying to learn to use moveit!, only to be told that I wasn't supposed to use moveit! (This was the majority of my week). 
- 4 days to write the rest of my subscriber, debug a bug in a standard node provided by ROS, and report those bugs to the ros maintainers, write my inverse kinematics solver and mimic dynamics, refactor code, restructure code base, find solutions for bugs that were encountered when running on ros-kinetic on ubuntu 16.04, and write this readme.

#### Payoff: Priceless. 
This was the first large project that I have completed since moving to USC this academic year, and I feel that I have a good understanding of ROS now, enough so that I can apply what I have learned in the RESL.

************************************
# Project Instructions
Node 1: MNIST publisher (C++)
- the node is supposed to read a key 0 .. 9 from the keyboard and publish the corresponding image of a digit from MNIST dataset.
- everytime the user presses a digit it should randomize digit from the corresponding digit class.
For example: if it is currently publishing 8 if you press 8 again it should select 8 with different writing.

- make sure that parameters are configurable in a yaml file (such as image topic, maybe other parameters if you need them)
- you can use rviz (from ROS) for digit image (topic) visualization


************************************
Node 2: digit tracker with a 2-link manipulator (python 3.5)
- the node subscribes to the MNIST image topic published by the MNIST publisher
- it runs an NN classifier using a pretrained TF model to classify 0 vs the rest of the digits
- if the digit 0 - extract contour corresponding to the digit writing and make a 2-link manipulator track this contour
- if the digit is not 0 - 'park' manipulator in the middle
- the manipulator can be simulated with the first order dynamics (i.e. a low pass filter on joint angle transitions), s.t. you would only have to care/solve inverse kinematics

- TF model should be completely read from a file, i.e. graph should not be reconstructed, but just read from pre-trained META file (but make sure you show me the training routine as well that should be a separate module from the node)
- TF model path, the topic for subscription, names of model outputs (tensors) that are used for classification should be configurable using yaml file (also add other parameters that you are using there, for example, the scale of the digit path, speed of tracking, constant of the manipulator dynamics, etc.)
- in rviz visualize the trajectory, the marker of the currently tracked position on the trajectory, transformations of the manipulator. 
- Ideally, you might want to also show the manipulator itself, i.e. create a URDF file with the manipulator robot (it is useful to know how the models are organized),
 but if you will be running out of time, at least make sure you publish all transforms and move manipulator parameters in the config yaml file (instead of URDF).

************************************
Overall:
- Please, use TF (i.e. no keras or another wrapper library)  
- make sure you write a HIGH QUALITY CODE and you document it very well. We will check code-base structuring as well.
- create detailed readme.md and put proper comments everywhere
- consider performance! (you can write some thoughts about possible bottlenecks and problems in readme.md)
- use standard code annotations for automatically generated documentation. You can check TF sources on how it is done for python (they use Sphinx, I believe)
- try to generate documentation from annotations of your code
Have fun and learn things, otherwise, what is the point? ;)
