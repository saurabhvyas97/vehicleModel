# Vehicle Model Simulation  

This repository contains a modular simulation model for vehicle dynamics, implemented in C. Each subsystem of the vehicle is designed as an independent module, promoting scalability and reusability. The project is structured to enable simulation of various vehicle subsystems, such as tire forces (more to be added).  

---

## Features  

- **Modular Design**: Subsystems are implemented as separate modules, such as `tireModel.c`, allowing for independent development and testing.  
- **Flexibility**: Easily extend the model by adding new subsystems or modifying existing ones.  
- **Interconnected Subsystems**: Outputs from one module serve as inputs for others, enabling dynamic interaction between vehicle components.  

---

## Repository Structure  

```plaintext
📦 vehicleModel  
├── 📂 src  
│   ├── tireModel.c     # Module for tire force calculations  
│   └── utils.c         # Common utility functions  
├── 📂 include  
│   ├── tireModel.h     # Header for tire model  
│   └── utils.h         # Header for utilities  
├── 📂 main  
│   ├── vehicleModel.c          # Main program for simulation  
├── Makefile            # Build automation script  
└── README.md           # Documentation  

