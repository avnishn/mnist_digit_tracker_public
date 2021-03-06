# Write 2 ROS nodes in a single ROS package. 

************************************
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
- put the code on github. Keep it in a private repo (github allows private repos for students, but you will have to apply for a special student pack, just google it), but share this repo with me and Yevgen. 
   - My github: @avnishn
- make sure you write a HIGH QUALITY CODE and you document it very well. We will check code-base structuring as well.
- create detailed readme.md and put proper comments everywhere
- consider performance! (you can write some thoughts about possible bottlenecks and problems in readme.md)
- use standard code annotations for automatically generated documentation. You can check TF sources on how it is done for python (they use Sphinx, I believe)
- try to generate documentation from annotations of your code
