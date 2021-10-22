# picopter
A quadcopter flight controller designed to run on a Raspberry Pi Zero W.

## Submodules
### `gobbledegook`
A far easier-to-use interface to the Bluetooth LE server API. See more details in its own separate repository.

## Building
This project uses GNU autotools to build. The root project requires no extra dependencies beyond GNU C++ compiler and autotools. However, the subproject `gobbledegook` has several other dependencies which can be found in its own repository.

As with most other projects, start by running configuration. For this project the working directory should be `<project root>/build`.
```
cd build
../configure
```

Then build the project using `make`.
```
make
```

The executable file `picopter` will then be built and placed in `<project root>/build`.
