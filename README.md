# ROS Biped Synchronizer

The goal of this node is to synchronize the image capture on a biped robot for various tasks, enabling the avoidance of parasitic oscillation induced by robot's walk.


## Requirements
- ROS 2 

```
                                +-------------------------------------------+
                +- RightFoot -->|--->+------------+                         |
  wrenchStamped |               |    | sync 0.01s | $|A-B|<270$?            |
                +- LeftFoot  -->|-+->+------------+     yes                 |
                                | |                      |                  |
                                | +-----------------+    +------------------|--> Trigger
                                |                   |    |                  |
                                |                 +------------+            |
   imageStamped [ image ------->|---------------->| sync 0.05s |------------|--> image
                                |                 +------------+            |
                                +-------------------------------------------+
```