# Vehicle Model Simulation  

This repository contains a modular simulation model for vehicle dynamics, implemented in C. Each subsystem of the vehicle is designed as an independent module, promoting scalability and reusability. The project is structured to enable simulation of various vehicle subsystems, such as tire forces, aerodynamics, and suspension.  

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
│   ├── main.c          # Main program for simulation  
│   ├── tireModel.c     # Module for tire force calculations  
│   ├── aeroModel.c     # Module for aerodynamic forces  
│   ├── suspension.c    # Module for suspension dynamics  
│   └── utils.c         # Common utility functions  
├── 📂 include  
│   ├── tireModel.h     # Header for tire model  
│   ├── aeroModel.h     # Header for aerodynamic model  
│   ├── suspension.h    # Header for suspension module  
│   └── utils.h         # Header for utilities  
├── 📂 tests  
│   ├── test_tireModel.c  # Unit tests for tire model  
│   ├── test_aeroModel.c  # Unit tests for aerodynamic model  
│   └── test_suspension.c # Unit tests for suspension module  
├── Makefile            # Build automation script  
└── README.md           # Documentation  

