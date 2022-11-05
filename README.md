# Robot-Learning-Control
Final Project for the course in Robotics 2 A.Y. 2019/2020.

## Project Description
In order to make a robot system execute tasks as regulation and trajectory tracking it needs definitely
a feedback control which effectively correct the errors of the feed-forward commands. Standard control
methods consist in the inversion of the dynamics model in order to calculate the necessary torque that
minimizes the error between the robot reference configuration and the actual one. A way more precise,
energy-efficient and suitable for complaint robots control method is the so-called Computed Torque which
corresponds to use the dynamic model over the system considering the joint weakly coupled. However, this
method requires to exactly know the dynamic robot model and therefore it is not an optimal neither a robust
control method. Therefore almost always we have some mismatch between the dynamic model (the nominal
one) and the actual behavior of the robot, also after identification procedures. In such cases it is convenient
to implement robot learning algorithms in order to better approximate the dynamic model and being able
apply a correction to the Feedback Linearization (FL) and so correctly linearize the system. In particular,
in this project, we are going to implement the Gaussian Process Regression over the corresponding torque
errors between the nominal dynamic models and the ”real one”.

## Documentation
A detailed [report](Robotics_II.pdf) of the project <br/>
The [presentation](Slides.pptx) we gave <br/>

## Resources used
[MATLAB Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)

## Authors
A. Mauro <br/>
G. Fioretti <br/>
E. Nicotra
