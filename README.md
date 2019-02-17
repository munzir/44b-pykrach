# 44-pykrang
Pyhthon interface for ach communication on krang

## Dependencies

- `python-dev libboost-python-dev`: i.e. in Ubuntu `sudo apt install python-dev libboost-python-dev`

- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions in the git readme.



## Build and Run

To compile

    mkdir build
    cd build
    cmake ..
    make

Lauch simulation using [41-krang-sim-ach](https://github.gatech.edu/WholeBodyControlAttempt1/41-krang-sim-ach), then

    python example.py
