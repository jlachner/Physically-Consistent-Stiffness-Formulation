# Physically-Consistent-Stiffness-Formulation 

## Description  
This repository contains code for the experiments related to the paper  
**"A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation"**  
by J. Lachner, M. C. Nah, and N. Hogan.

## Repository Structure  

- `/FRI_ClientApplication/` – Client-side application for interfacing with the KUKA LBR robot  
  - `/trajectory_csv/` – Recorded joint and end-effector trajectory data  
  - `/src/` – Source code for implementing the stiffness formulation  
    - `MyLBRClient.cpp/.h` – Core implementation for the LBR robot client  
    - `my_diff_jacobians.cpp/.h` – Differential Jacobians for stiffness computation  
    - `MyLBRApp.cpp` – Application entry point  
    - `CMakeLists.txt` – Build configuration for CMake  

- `/Matlab/` – MATLAB scripts for analysis, visualization, and validation  
  - `/Experiment1/` – Scripts for anthropomorphic model validation  
  - `/Experiment2/` – Scripts for IIWA14 contact-rich manipulation experiments  
  - `main_CS.m` – Main script for contact-stiffness analysis (replaces old `main_Antropomorphic.m`)

- `LICENSE` – Licensing information  
- `README.md` – This document  

## Requirements  

### C++  
- **Eigen** (matrix computations)  
- **Boost** (threading and timing utilities)  
- **Explicit-cpp** (kinematic and dynamic calculations for stiffness formulation)  

### MATLAB  
- MATLAB R2020b or newer  
- Explicit-MATLAB (for data processing and analysis)  

## Explicit-MATLAB & Explicit-FRI  
This repository utilizes **Explicit-MATLAB** and **Explicit-FRI** for robot interaction and analysis.  
For more information, visit: [Explicit Robotics](https://explicit-robotics.github.io/).

## Citation  
If you use this repository, please cite our paper:

    J. Lachner, M. C. Nah, and N. Hogan, "A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation," arXiv, 2025.