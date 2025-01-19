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
```
---

## Prerequisites  

- **Compiler**: A C compiler such as GCC.  
- **Build System**: Make (recommended for using the provided Makefile).  
- **Development Environment**: Tested on Linux and Windows.  

---

## Getting Started  

### 1. Clone the Repository  
```bash
git clone https://github.com/saurabhvyas97/vehicleModel.git  
cd vehicleModel  
```

### 2. Build the Project

Run the following command to compile the project:
```bash
make
```

### 3. Run the Simulation
Execure the program:
```bash
./build/vehicleModel
```

---

## Modules

#### Tire Model (tireModel.c)
Calculates tire forces (Lateral Fy) based on:
- Slip angle
- Normal load

---

## Contributions
Contributions are welcome!

To contribute:
1. Fork the repository.

2. Create a new branch:
```bash
git checkout -b feature/YourFeature
```

3. Commit yor changes:
```bash
git commit -m "Add new feature"
```

4. Push to the branch:
``bash
git push origin feature/YourFeature
```

5. Open a pull request