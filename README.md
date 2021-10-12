# RT Manipulators CPP library and examples

## Installation


### Install Dynamixel SDK

https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux

```sh
$ sudo apt install build-essential
$ cd ~
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ cd ~/DynamixelSDK/c++/build/linux64
$ make
$ sudo make install
```

### Install other dependencies

```sh
$ sudo apt install libyaml-cpp-dev 
```

### Build RT Manipulators CPP

```sh
$ git clone https://github.com/rt-net/rt_manipulators_cpp
$ cd rt_manipuators_cpp

$ ./make.bash
```

## Usage

### Run test

```sh
$ cd rt_manipuators_cpp
$ ./run_test.bash
```
