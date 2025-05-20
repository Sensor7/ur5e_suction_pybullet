# UR5 with Epick Gripper: Object Pick and Place Simulation

This project simulates a UR5 robotic arm with a Epick suction gripper, performing autonomous object grasping and placement tasks in the PyBullet environment. Using inverse kinematics (IK) for precise arm control and synchronized joint control for realistic gripper motion, the robot grasps cubes from random positions and places them on a tray.


This repository is mainly adapt from [ur5_grasp_object_pybullet](https://github.com/leesweqq/ur5_grasp_object_pybullet), change the urdf for gripper and also add the bin.
And it's like a prototype for fast checking the pipeline. And because pybullet is very easy to use and can quickly build a prototype. So I choose it as the first simulator.

## Results Showcase  

### Grasping and Placing Demo  
In repository demo folder

---

## Usage Instructions  

### 1. Setup the Environment  
- Install the required dependencies:  
    ```bash  
    pip install pybullet  

### 2. Run the Simulation
- This will initialize the simulation, and the robot will begin performing the object grasping and placement task.
- Launch the program using the following command:
    ```bash
    python ur5_epick.py

### 3. PyBullet GUI
- Once the simulation is running, you can use the PyBullet GUI to observe the robot’s actions in real-time. The interface allows you to track the movement of the robotic arm, gripper, and cubes as they interact within the environment.

- If you meet such issue

```
pybullet build time: Jan 29 2025 23:16:28
startThreads creating 1 threads.
starting thread 0
started thread 0 
argc=2
argv[0] = --unused
argv[1] = --start_demo_name=Physics Server
ExampleBrowserThreadFunc started
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
Creating context
Failed to create GL 3.3 context ... using old-style GLX context
Failed to create an OpenGL context
```

Then follow the sulution:

```
sudo prime-select nvidia
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep "OpenGL"
```

- TO ensure the simulation can be run, should be use gpu if there is no mesa in your PC

```
export LIBGL_ALWAYS_INDIRECT=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

---

## Key Features

- **UR5 Robotic Arm Simulation**: The UR5 robotic arm uses inverse kinematics (IK) to move to the correct positions and accurately control its joints for task execution.
- **Epick suction Gripper**: The gripper is simulated using create/detach joint constrain to monitor the suction on/off command
- **Dynamic Object Placement**: Cubes are randomly generated in the environment, making the task more dynamic and representative of real-world applications.
- **Real-Time Interaction**: The PyBullet GUI allows for interactive and visual monitoring of the robot’s actions, providing a comprehensive view of the simulation.

---

## Resources & References

- **PyBullet**: PyBullet is a physics engine for simulating robots and their environments. You can find more details and documentation on the official website of [PyBullet](https://pybullet.org/).


