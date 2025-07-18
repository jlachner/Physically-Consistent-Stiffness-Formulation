# Physically-Consistent-Stiffness-Formulation 

## Description  
This repository contains code for the experiments related to the paper  
**"A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation"**  
by J. Lachner, M. C. Nah, and N. Hogan.

## Repository Structure  

- `/FRI_ClientApplication/` – Client-side application for interfacing with the KUKA LBR robot  
  - `/trajectory_csv/` – Recorded joint trajectory data  
  - `/src/` – Source code for implementing the stiffness formulation  
    - `MyLBRClient.cpp/.h` – Core implementation for the LBR robot client  
    - `my_diff_jacobians.cpp/.h` – Differential Jacobians for stiffness computation  
    - `MyLBRApp.cpp` – Application entry point  
    - `CMakeLists.txt` – Build configuration for CMake  

- `/Matlab/` – MATLAB scripts for analysis, visualization, and validation  
  - `/Experiment1/` – Scripts for Experiment 1: Parkour
  - `/Experiment2/` – Scripts for Experiment 1: Wiping a bowl  

- `LICENSE` – Licensing information  
- `README.md` – This document  

## Requirements  

### C++  
- **Eigen** (matrix and vector computations)  
- **Boost** (threading and timing utilities)  
- **[Explicit-FRI](https://github.com/explicit-robotics/Explicit-FRI)** (KUKA Fast Research Interface implementation)

### MATLAB  
- MATLAB R2020b or newer  
- Explicit-MATLAB (for data processing and analysis) 
- **[Explicit-MATLAB](https://github.com/explicit-robotics/Explicit-MATLAB)** 

## Explicit-MATLAB & Explicit-FRI  
This repository utilizes **Explicit-MATLAB** and **Explicit-FRI** for robot interaction and analysis.  
For more information, visit: [Explicit Robotics](https://explicit-robotics.github.io/).

## Citation  
If you use this repository, please cite our paper:

    J. Lachner, M. C. Nah, and N. Hogan, "A Physically Consistent Stiffness Formulation for Contact-Rich Manipulation," arXiv, 2025.