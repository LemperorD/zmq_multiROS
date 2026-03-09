# zmq_multiROS

a C++ package used to provide communication between multiple ROS/ROS2 PC

> Initially, the project will focus on local area network communication using zmqpp.

> 🚧: the project is coming soon...

# A. Install

### 1. Clone to your PC

```bashrc
git clone https://github.com/LemperorD/zmq_multiROS.git
```

### 2. Install ROS/ROS2 (Robot Operating System)

[ROS Install Guide](https://wiki.ros.org/Installation/Ubuntu) 
/ [ROS2 Install Guide](https://docs.ros.org/en/jazzy/Installation.html)

But I **strongly recommend to use the fishros** to install.

```bashrc
wget http://fishros.com/install -O fishros && . fishros
```

### 3. Install libzmq & zmqpp

#### a. libzmq
```bash
sudo apt install libzmq3-dev
```

#### b. zmqpp
```bash
mkdir ~/tools # u can use other folds to save the repo
git clone https://github.com/zeromq/zmqpp.git
cd zmqpp
mkdir build && cd build 
cmake ..
make -j4
sudo make install
```

## B. ToDolist:
- [X] doc to install ROS/ROS2
- [X] doc to install libzmq & zmqpp
- [ ] subscriber/publisher
- [ ] server/client
- [ ] actions

## Reference

✨ The inspiration comes form this project: https://github.com/shupx/swarm_ros_bridge.git

## why i do this

This project originally stemmed from a confidential project, and I aimed at **an independent package** to provide communication between drone swarms in local area network. 

It was built upon **a messy API code💩** from a senior, and a lot of effort went into modifying it. 

At the time, I thought I could open-source the communication part, but **my supervisor didn't allow it**😢. 

Nevertheless, I still want to make this technology available to the open-source community.
