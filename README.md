# 44b-pykrach
Python interface for ach communication on krang

## Dependencies

- `python-dev`, `libboost-python-dev`

      sudo apt install python-dev libboost-python-dev

- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic)
 Install the repo.

### Optional Dependency

For simulation:

- [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach)
 Install the repo.

## Build and Run

    mkdir build
    cd build
    cmake ..
    make

Lauch simulation using [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach), then

    python example.py
