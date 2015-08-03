# HOW TO RUN

1. Make sure you have the correct packages installed for the NAO to work with ROS. They can be found in the tutorial here: http://wiki.ros.org/nao/Tutorials/Installation

2. The package that contains the correct nodes to launch are found in nao_bringup. If you are unsure if you have the correct package installed, you can type ```roscd nao_bringup``` which should navigate you to the package, if installed properly.  If not, go through the steps of installation again. In the nao_bringup package there should be a launch file called nao.launch. This is what you will be launching when you want to use the NAO with ROS.

3. Once you are sure you have the correct package and launch file, make sure that the workspace is sourced. If you are unsure of how to go about this, the process of how to source workspaces can be found here: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

4. To launch correctly, you must know what the NAO's IP address is. To retrieve it, double tap the chest button. The NAO should say something along the lines of "Hello, my name is NAO (or whatever name he is given). My internet address is 10.0.3.16" (or whatever IP he has). 

5. Now that you have the launch file and the IP address, you can go a head and start up the nodes. To launch the file, you must type in:

	```NAO_IP=10.0.3.16 roslaunch nao_bringup nao.launch```

   This will launch every node that is needed to get the demo working.

6. To launch the demo, make sure that the workspace where the demo is located is sourced, much like you did in step 2, but with the demo package now. 

7. Once everything is sourced, you can launch the launch file. The launch file can be found in the statepublisher package. To launch it type:

	```roslaunch statepublisher states.launch```

   This should launch the walk_detector, set_pose, stand_up_fd, stand_up_fu, and statepublish_node nodes.

8. The NAO should now be trying to walk, get up, etc... if everything worked properly.

9. There are other various programs that were written for the NAO. They can all be found on this repository. For example, if you want the NAO to do some math, type:

	```rosrun functions mathp```

   To find the other programs, just look through the src files/CMakelists to find the package name (name of the folder with a CMakeList file and a src folder in it) and node names (name of the .c file in the package src file), running each with the pattern ```rosrun (package name) (node name)```.

# CHANGES TO MAKE
Need to change the statepublihser to call services instead of publishing messages 
to make the code more fluid and better to understand; right now the publisher
publishes a message as true and then waits for another node to publish it as false instead of
just waiting for a service to return.

# What's With All The Commits?
The reason as to why there are so many commits is that my partner, Victoria, could not connect
to the NAO or build on her computer, so it needed to all be done on mine. To do this,
I always had to commit before I merged in case anything went wrong, then I had
to pull her branch into mine and build from there. Often times, we needed to make small
changes just to see how changing a variable from 0.1 to 0.3 would affect how the robot responds. 
To do this Victoria would work on her computer, change a few things while I edited other things,
commit, merge, and build on mine. There are way too many commits, but this is the reason as to why.


# NAO Workspace
Contains all the information and code associated with the NAO robot at the UML Robotics Lab

# Authors
Alexander Infantino
Victoria Albanese

# WIP
Last Updated July 7, 2015

# WOW THIS CODE IS CRAP
Yes, we are aware that most of our code could be condensed and organized much better, but we are still newbies
in the computer science field. This stuff will come with time, and hopefully the code will look much 
better in the future.
