# Robot Probabilistic Localization

This repository contains the implementation of a probabilistic localization algorithm for robots. The goal of this project is to estimate the position of a robot in a known environment using sensor data and probabilistic methods. The algorithm is based on Bayesian filtering, specifically the **Histogram Filter** (also known as Discrete Bayes Filter), which is a simple yet effective method for localization in discrete state spaces.

## Table of Contents
1. [Overview](#overview)
2. [Dependencies](#dependencies)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Algorithm](#algorithm)
6. [Contributing](#contributing)
7. [License](#license)

---

## Overview

Probabilistic localization is a fundamental problem in robotics, where the robot must estimate its position in a known environment using noisy sensor data. This repository provides a Python implementation of a **Histogram Filter** for 1D and 2D grid-based environments. The filter updates the robot's belief about its position over time as it moves and senses its surroundings.

Key features:
- 1D and 2D grid-based localization.
- Motion and sensor models for probabilistic updates.
- Visualization of the robot's belief over time.

---

## Dependencies

To run this project, you need the following Python libraries:
- `numpy`: For numerical computations.
- `matplotlib`: For visualizing the robot's belief over the grid.

You can install the dependencies using `pip`:
```bash
pip install numpy matplotlib
```

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/FlipGoncalves/RobotProbabilisticLocalization.git
   cd RobotProbabilisticLocalization
   ```

2. Install the required dependencies (see [Dependencies](#dependencies)).

---

## Usage

The repository contains the following main files:
- `1D_localization.py`: Implements probabilistic localization in a 1D grid environment.
- `2D_localization.py`: Extends the algorithm to a 2D grid environment.

To run the 1D localization example:
```bash
python 1D_localization.py
```

To run the 2D localization example:
```bash
python 2D_localization.py
```

### Customization
You can modify the following parameters in the code:
- Grid size: Change the size of the environment.
- Motion and sensor noise: Adjust the probabilities in the motion and sensor models.
- Robot movements and measurements: Simulate different paths and sensor readings.

---

## Algorithm

The probabilistic localization algorithm is based on the **Histogram Filter**, which consists of two main steps:
1. **Prediction Step**: Update the robot's belief based on its motion model.
2. **Correction Step**: Update the belief based on sensor measurements using Bayes' rule.

### Motion Model
The motion model describes how the robot's position changes over time. It is represented as a probability distribution over possible new states given the current state and the robot's action.

### Sensor Model
The sensor model describes the likelihood of receiving a particular sensor reading given the robot's true state. It is used to update the belief during the correction step.

### Visualization
The robot's belief is visualized as a histogram over the grid. As the robot moves and senses, the belief distribution is updated and displayed.

---

## Contributing

Contributions are welcome! If you'd like to contribute to this project, please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes and push to your branch.
4. Submit a pull request with a detailed description of your changes.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- This project is inspired by the probabilistic robotics concepts presented in the book *Probabilistic Robotics* by Sebastian Thrun, Wolfram Burgard, and Dieter Fox.
- Special thanks to the open-source community for providing valuable tools and libraries.

---

For any questions or feedback, feel free to open an issue or contact the repository owner. Happy coding! 🤖
