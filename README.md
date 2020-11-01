# Persistent-ORB-SLAM
**Authors:** [Zakieh S Hashemifar](http://www.buffalo.edu/~zakiehsa), [Karthik Dantu](https://cse.buffalo.edu/faculty/kdantu/)

Persistent-ORB-SLAM is an extended version of ORB-SLAM which incorporates persistence estimation for features to evaluate their existence in the environment over time and removes the features which are not available anymore. This new features makes ORB_SLAM a better fit for dynamic environments and over long runs.

### Related Publications:

Zakieh S. Hashemifar and Karthik Dantu. [**Practical Persistence Reasoning in Visual SLAM**.](https://ieeexplore.ieee.org/abstract/document/9196913) *2020 IEEE International Conference on Robotics and Automation (ICRA).*

### Building Instructions

For some of the source files, there are some numbered versions from 1 to 4. These source files include Tracking.cc, LoopClosing.cc, Optimizer.cc, ORBmatcher.cc.
The numbered vesions of these files provide step by step incorporation of persistence filter into ORBSLAM.

Step 1: Incorporating vanilla persistence filter into ORB_SLAM as described in their [paper](https://ieeexplore.ieee.org/document/7487237) without any further changes.

Step 2: Add translational and rotational constraints to the persistence to avoid removing persistence features.

Step 3: Add occlusion checking capability, so less false negative observations would be assigned to features.

Step 4: Provide finer search for features with persistence probability higher than specified threshold (0.8 by default), so more true positive observations would be encountered.

In order to use either of these versions, the corresponding numbered version of those source files should be used.
For example if step 2 is requested, Tracking_step2.cc should be renamed to Tracking.cc, LoopClosing_step2.cc should be renamed to LoopClosing.cc and etc.
The rest of the build could be followed from origin [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) github page.
