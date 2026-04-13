# OptiSwarmCF

## Setup

### 1. Backend
cd backend_ros2
colcon build
source install/setup.bash

### 2. SDK
cd sdk
pip install -e .

### 3. Run example
python examples/first_ex_cf.py