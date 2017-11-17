## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./derive_theta
[image2]: ./derive_theta

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To derive the DH parameters I followed along with the KR210 Forward Kinematics demo, drawing the same diagram and deriving the appropriate parameters from the kr210.urdf.xacro file.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Using the DH parameters given above, I created the individual transformation matrices.  This is done in lines 43-110 of my IK_server.py code, where I took the DH parameter table and inserted values into the transformation matrix between adjacent links developed in the "Forward Kinematics" lesson.  The composition of homogeneous transforms to get from the origin to the gripper frame is then defined by multiplying all transformation matrices between each link together.     


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In order to derive theta 1-3 I referenced the inverse kinematics example from lesson 2 as well as the "Inverse Kinematics with Kuka KR210".  Theta 1 was the easy part, due to just needing to project the x and y coordinates from the wrist center.  However, thetas 2 and 3 were much more challenging.  The "Inverse Kinematics with Kuka KR210" lesson was extremely helpful in visualizing this.  I followed along and created my own sketch shown below to better understand the approach, as well as to determine the equations that would need to go into the python code.  

![alt text][image2]

To determine theta 4-6, as established in the lesson, we first establish the transformation between joints 0 and 3 (R0_3) and then multiply the inverse of this by the total system transformation matrix.  The resulting matrix is the transform between joints 3 and 6 (R3_6), from which we can derive theta 4-6 as covered in lesson 2, section 8.

'''
 theta4 = atan2(R3_6[2,2], -R3_6[0,2])
 theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
 theta6 = atan2(-R3_6[1,1], R3_6[1,0])
'''


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Overall I was able to reach the goal of the robot completing 8/10 pick and place cycles.  Lines 31-126 of my IK_server.py contain the forward kinematics section, while lines 130-213 contain the inverse kinematics.  One of the mistakes I made initially was containing my forward kinematics within the for loop, which forced it to re-calculate every time.  This led to minutes of run time just to complete one pick and place operation.  To speed up the process I did move the forward kinematics outside of the for loop, as suggested in the walk-through video.  I did also make the mistake of using "simplify" for every transform.  This became apparent when I used the IK_debug script and the inverse kinematics where taking nearly a minute to calculate, versus less than a second.  Getting rid of this significantly improved the speed of the overall pick and place operation.  One of the issues I attempted to, but have yet to solve, is the excessive wrist rotation as the arm moves to the pick up location.  Although the arm eventually gets to the correct location to make the pickup, this significantly slows down the process and is something I would like to improve.  Now that I have the framework established for completing the pick and place operation, my future improvements can mainly be centered around improved performance. 

Overall I found this project interesting, yet quite challenging.  I found myself endlessly referencing the lessons on inverse/forward kinematics as well as the walk through video when I got stuck.  It was therefore extremely gratifying to see the object get picked up and dropped in the bin successfully the first time.     
