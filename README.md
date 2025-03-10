# Physically-Consistent-Stiffness-Formulation 

## Description  
This repository contains code for the experiments related to the paper "A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation" by J. Lachner, M. C. Nah, and N. Hogan. 

## Repository Structure  
- `/FRI_ClientApplication/` - Client-side application for interfacing with the robot  
  - `/data/` - Contains recorded joint trajectory data (`q_recorded.csv`)  
  - `/src/` - Source code for implementing the stiffness formulation  
    - `MyLBRClient.cpp/.h` - Core implementation for the LBR robot client  
    - `my_diff_jacobians.cpp/.h` - Code for computing differential Jacobians  
    - `MyLBRApp.cpp` - Application entry point  
    - `CMakeLists.txt` - Build configuration for CMake  
- `/Matlab/` - MATLAB scripts for additional analysis and visualization  
  - `main_Antropomorphic.m` - Main script for anthropomorphic model analysis  
  - `main_bowl_iiwa14.m` - MATLAB script for IIWA 14 robot bowl task  
- `LICENSE` - Licensing information  
- `README.md` - This document  

## Requirements  
To run the experiments, install the required dependencies:

- **C++ Libraries**:
  - Eigen (for matrix computations)
  - Boost (for threading and chrono utilities)
  - Explicit-cpp (for kinematic and dynamic calculations)

- **MATLAB** (for analysis scripts)

## Explicit-MATLAB & Explicit-FRI  
This repository utilizes **Explicit-MATLAB** and **Explicit-FRI** for robot interaction and analysis.  
For more information, visit: [Explicit Robotics](https://explicit-robotics.github.io/).

## Citation  

If you use this repository, please cite our paper:

    J. Lachner, M. C. Nah, and N. Hogan, "A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation," arXiv, 2025.