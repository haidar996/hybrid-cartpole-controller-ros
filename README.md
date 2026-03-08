# Hybrid CartPole Controller (ROS)

Hybrid control system for stabilizing and controlling the CartPole (Inverted Pendulum) problem using **Robot Operating System (ROS)**.

---

## 📌 Overview

This project implements a **hybrid controller** for the CartPole system.  
The CartPole is a classical control problem where the goal is to balance a pole upright on a moving cart by applying horizontal forces.

The controller architecture combines control strategies to improve stability, robustness, and performance.

The repository includes:

- ROS package implementation
- Controller source code
- Launch files
- Performance plots
- Demonstration video

---

## 🧠 System Description

The CartPole system consists of:

- A cart that moves horizontally
- A pole attached to the cart via a joint
- The objective: keep the pole balanced upright (θ = 0)

This project focuses on:

- Stabilization control
- State feedback control
- Hybrid control logic
- Simulation and visualization inside ROS

---

## 📁 Repository Structure
```
hybrid-cartpole-controller-ros/
│
├── src/
│   └── cart_pole/          # ROS package
│       ├── launch/         # Launch files
│       ├── src/            # Source code
│       ├── include/        # Header files (if C++)
│       └── package.xml
│
├── plots/                  # Performance graphs and results
├── video.mp4               # Demonstration video
├── README.md
└── CMakeLists.txt
```

---

## ⚙️ Requirements

- ROS (Noetic recommended for ROS1)
- Catkin workspace
- C++ or Python (depending on implementation)
- Standard ROS dependencies

---

## 🔧 Installation & Build

### 1️⃣ Create Workspace (if not already created)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2️⃣ Clone Repository
```bash
git clone https://github.com/haidar996/hybrid-cartpole-controller-ros.git
```

### 3️⃣ Build Workspace
```bash
cd ~/catkin_ws
catkin_make
```

### 4️⃣ Source Setup File
```bash
source devel/setup.bash
```

---

## ▶️ Running the Project

Use the provided launch files:
```bash
roslaunch cart_pole <launch_file_name>.launch
```

Replace `<launch_file_name>` with the actual file inside the `launch/` folder.

---

## 📊 Results

The repository includes:

- Performance plots inside the `plots/` folder
- System response graphs
- Control behavior visualization
- Demonstration video (`video.mp4`)

These results show:

- Pole angle stabilization
- System response over time
- Controller effectiveness

---

## 🎯 Project Goals

- Stabilize the inverted pendulum
- Reduce oscillations
- Improve robustness
- Demonstrate hybrid control in ROS
- Provide a clean and reusable ROS implementation

---

## 🚀 Future Improvements

Possible extensions:

- Add reinforcement learning controller
- Compare with LQR or PID
- Add Gazebo simulation integration
- Improve tuning and performance analysis
- Add real-robot implementation

---

## 🤝 Contributing

Contributions are welcome. To contribute:

1. Fork the repository
2. Create a new branch
3. Make changes
4. Submit a Pull Request

---

## 📜 License

Add your preferred license (MIT, GPL, etc.) in a `LICENSE` file.

---

## 👨‍💻 Author

Developed by **Haidar Saad**  
GitHub: [https://github.com/haidar996](https://github.com/haidar996)
