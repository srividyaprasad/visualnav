# General Navigation Models: GNM, ViNT and NoMaD

**Contributors**: Dhruv Shah, Ajay Sridhar, Nitish Dashora, Catherine Glossop, Kyle Stachowicz, Arjun Bhorkar, Kevin Black, Noriaki Hirose, Sergey Levine

_Berkeley AI Research_

[Project Page](https://general-navigation-models.github.io) | [Citing](https://github.com/robodhruv/visualnav-transformer#citing) | [Pre-Trained Models](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing)

---

General Navigation Models are general-purpose goal-conditioned visual navigation policies trained on diverse, cross-embodiment training data, and can control many different robots in zero-shot. They can also be efficiently fine-tuned, or adapted, to new robots and downstream tasks. Our family of models is described in the following research papers (and growing):
1. [GNM: A General Navigation Model to Drive Any Robot](https://sites.google.com/view/drive-any-robot) (_October 2022_, presented at ICRA 2023)
2. [ViNT: A Foundation Model for Visual Navigation](https://general-navigation-models.github.io/vint/index.html) (_June 2023_, presented at CoRL 2023)
3. [NoMaD: Goal Masking Diffusion Policies for Navigation and Exploration](https://general-navigation-models.github.io/nomad/index.html) (_October 2023_)

## Overview
This repository contains pre-trained model checkpoints, as well as example code to deploy it on a TurtleBot3 robot in Isaac Sim. The repository follows the organization from [GNM](https://github.com/PrieureDeSion/drive-any-robot).

- `./train/vint_train/models/`: contains model files for GNM, ViNT, and some baselines.
- `./deployment/src/record_bag.sh`: script to collect a demo trajectory as a ROS bag in the target environment on the robot. This trajectory is subsampled to generate a topological graph of the environment.
- `./deployment/src/create_topomap.sh`: script to convert a ROS bag of a demo trajectory into a topological graph that the robot can use to navigate.
- `./deployment/src/navigate.sh`: script that deploys a trained GNM/ViNT/NoMaD model on the robot to navigate to a desired goal in the generated topological graph. Please see relevant sections below for configuration settings.
- `./deployment/src/explore.sh`: script that deploys a trained NoMaD model on the robot to randomly explore its environment. Please see relevant sections below for configuration settings.

## Deployment
This subfolder contains code to load a pre-trained ViNT and deploy it on the open-source [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) in [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html). It can be easily adapted to be run on alternate robots, and researchers have been able to independently deploy it on the following robots – Clearpath Jackal, DJI Tello, Unitree A1, TurtleBot2, Vizbot – and in simulated environments like CARLA.

### Simulation Setup

This software was tested on TurtleBot3 in Isaac Sim running Ubuntu 22.04.

#### Software Installation (in this order)
1. [Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html)
2. [ROS2 Humble](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros)
3. TurtleBot3 Setup

    Spawn the robot in Isaac Sim by importing a URDF. Refer to [URDF Import: Turtlebot](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_turtlebot.html)

    Create a ROS2 Omnigraph using GUI or Python Scripting. 
    - Refer to [Driving TurtleBot via ROS2 messages](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html) for setting up the drive `/cmd_vel `subscriber. 
    - Refer to [ROS2 Cameras](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_camera.html) for setting up the camera `/rgb` publisher. 
    - Optionally, refer to [Setting up Odometry](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_tf.html#setting-up-odometry) for setting up the `/odom` publisher.

4. Virtual Environment 
    - Make venv (run this inside the `vint_release/` directory)
        ```bash
        python3 -m venv .venv
        ```
    - Source env 
        ```bash
        source .venv/bin/activate
        ```
    - Install required packages
        ```bash
        pip install -r requirements.txt
        ```

5. Install the `vint_train` packages (run this inside the `vint_release/` directory):
    ```bash
    pip install -e train/
    ```
6. Install the `diffusion_policy` package from this [repo](https://github.com/real-stanford/diffusion_policy):
    ```bash
    git clone git@github.com:real-stanford/diffusion_policy.git
    pip install -e diffusion_policy/
    ```
7. (Recommended) Install [tmux](https://github.com/tmux/tmux/wiki/Installing) if not present.
    Many of the bash scripts rely on tmux to launch multiple screens with different commands. This will be useful for debugging because you can see the output of each screen.

### Loading the model weights

Save the model weights *.pth file in `vint_release/deployment/model_weights` folder. Our model's weights are in [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).

### Collecting a Topological Map

_Make sure to run these scripts inside the `vint_release/deployment/src/` directory._


This section discusses a simple way to create a topological map of the target environment for deployment. For simplicity, we will use the robot in “path-following” mode, i.e. given a single trajectory in an environment, the task is to follow the same trajectory to the goal. The environment may have new/dynamic obstacles, lighting variations etc.

#### Record the rosbag: 
```bash
./record_bag.sh <bag_name>
```

Run this command to teleoperate the robot with the joystick and camera. This command opens up a pane and another terminal:
1. `ros2 bag record /rgb -o <bag_name>`: This command isn’t run immediately (you have to click Enter). It will be run in the vint_release/deployment/topomaps/bags directory, where we recommend you store your rosbags.
2. `teleop_twist_keyboard`: This node reads inputs from the keyboard and outputs them on /cmd_vel topic that teleoperates the robot’s base.

Once you are ready to record the bag, run the `ros2 bag record` script and teleoperate the robot on the map you want the robot to follow. When you are finished with recording the path, kill the `ros2 bag record` command, and then kill the tmux session.

#### Make the topological map: 
```bash
./create_topomap.sh <topomap_name> <bag_filename>
```

This command opens up 2 panes:
1. `python create_topomap.py —dt 1 —dir <topomap_dir>`: This command creates a directory in `/vint_release/deployment/topomaps/images` and saves an image as a node in the map every second the bag is played.
2. `ros2 bag play -r 1.5 <bag_filename>`: This command plays the rosbag at x5 speed, so the python script is actually recording nodes 1.5 seconds apart. The `<bag_filename>` should be the entire bag name with the .bag extension. You can change this value in the `make_topomap.sh` file. The command does not run until you hit Enter, which you should only do once the python script gives its waiting message. Once you play the bag, move to the screen where the python script is running so you can kill it when the rosbag stops playing.

When the bag stops playing, kill the tmux session.


### Running the model 
#### Navigation
_Make sure to run this script inside the `vint_release/deployment/src/` directory._

```bash
./navigate.sh “--model <model_name> --dir <topomap_dir>”
```

To deploy one of the models from the published results, we are releasing model checkpoints that you can download from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


The `<model_name>` is the name of the model in the `vint_release/deployment/config/models.yaml` file. In this file, you specify these parameters of the model for each model (defaults used):
- `config_path` (str): path of the *.yaml file in `vint_release/train/config/` used to train the model
- `ckpt_path` (str): path of the *.pth file in `vint_release/deployment/model_weights/`


Make sure these configurations match what you used to train the model. The configurations for the models we provided the weights for are provided in yaml file for your reference.

The `<topomap_dir>` is the name of the directory in `vint_release/deployment/topomaps/images` that has the images corresponding to the nodes in the topological map. The images are ordered by name from 0 to N-1.

This command opens up 2 panes:

1. `python navigate.py --model <model_name> -—dir <topomap_dir>`: This python script starts a node that reads in image observations from the `/rgb` topic, inputs the observations and the map into the model, and publishes actions to the `/waypoint` topic.
2. `python pd_controller.py`: This python script starts a node that reads messages from the `/waypoint` topic (waypoints from the model) and outputs velocities to navigate the robot’s base.

When the robot is finishing navigating, kill the `pd_controller.py` script, and then kill the tmux session. 

#### Exploration
_Make sure to run this script inside the `vint_release/deployment/src/` directory._

```bash
./exploration.sh
```

To deploy one of the models from the published results, we are releasing model checkpoints that you can download from [this link](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing).


The `<model_name>` is the name of the model in the `vint_release/deployment/config/models.yaml` file (note that only NoMaD works for exploration). In this file, you specify these parameters of the model for each model (defaults used):
- `config_path` (str): path of the *.yaml file in `vint_release/train/config/` used to train the model
- `ckpt_path` (str): path of the *.pth file in `vint_release/deployment/model_weights/`


Make sure these configurations match what you used to train the model. The configurations for the models we provided the weights for are provided in yaml file for your reference.

The `<topomap_dir>` is the name of the directory in `vint_release/deployment/topomaps/images` that has the images corresponding to the nodes in the topological map. The images are ordered by name from 0 to N-1.

This command opens up 2 panes and a separate terminal:

1. `python explore.py`: This python script starts a node that reads in image observations from the `/rgb` topic, inputs the observations and the map into the model, and publishes exploration actions to the `/waypoint` topic.
2. `python pd_controller.py`: This python script starts a node that reads messages from the `/waypoint` topic (waypoints from the model) and outputs velocities to navigate the robot’s base.
3. `teleop_twist_keyboard`: This python script starts a node that reads inputs from the keyboard and outputs them on `/cmd_vel` topic that teleoperates the robot’s base.

When the robot is finishing exploring, kill the `pd_controller.py` script, and then kill the tmux session. If you want to take control of the robot while it is exploring, the `teleop_twist_keyboard` node allows you to do so with your keyboard.


### Adapting this code to different robots

We hope that this codebase is general enough to allow you to deploy it to your favorite ROS-based robots. You can change the robot configuration parameters in `vint_release/deployment/config/robot.yaml`, like the max angular and linear velocities of the robot and the topics to publish to teleop and control the robot. Please feel free to create a Github Issue or reach out to the authors at shah@cs.berkeley.edu.


## Citing
```
@inproceedings{shah2022gnm,
  author    = {Dhruv Shah and Ajay Sridhar and Arjun Bhorkar and Noriaki Hirose and Sergey Levine},
  title     = {{GNM: A General Navigation Model to Drive Any Robot}},
  booktitle = {International Conference on Robotics and Automation (ICRA)},
  year      = {2023},
  url       = {https://arxiv.org/abs/2210.03370}
}

@inproceedings{shah2023vint,
  title     = {Vi{NT}: A Foundation Model for Visual Navigation},
  author    = {Dhruv Shah and Ajay Sridhar and Nitish Dashora and Kyle Stachowicz and Kevin Black and Noriaki Hirose and Sergey Levine},
  booktitle = {7th Annual Conference on Robot Learning},
  year      = {2023},
  url       = {https://arxiv.org/abs/2306.14846}
}

@article{sridhar2023nomad,
  author  = {Ajay Sridhar and Dhruv Shah and Catherine Glossop and Sergey Levine},
  title   = {{NoMaD: Goal Masked Diffusion Policies for Navigation and Exploration}},
  journal = {arXiv pre-print},
  year    = {2023},
  url     = {https://arxiv.org/abs/2310.xxxx}
}
```
