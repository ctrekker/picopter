# picopter
A quadcopter flight controller designed to run on a Raspberry Pi Zero W.

## Submodules
### `gobbledegook`
A far easier-to-use interface to the Bluetooth LE server API. See more details in its own separate repository.

### `eigen`
Very convenient library for linear algebra. Most of its use in this project is for vector and quaternion math.

## Building
This project uses GNU autotools to build. The root project requires no extra dependencies beyond GNU C++ compiler and autotools. However, the subproject `gobbledegook` has several other dependencies which can be found in its own repository.

First download the repository along with its submodules. The `--recursive` flag is important since it tells git to download each git submodule's respective repository along with this one.
```
git clone --recursive https://github.com/ctrekker/picopter.git
```

As with most other autotools projects, start by running configuration. For this project the working directory should be `<project root>/build`.
```
cd build
../configure
```

Then build the project using `make`.
```
make
```

The executable file `picopter` will then be built and placed in `<project root>/build`.
