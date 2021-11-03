AUTOMAKE_OPTIONS = subdir-objects
noinst_LIBRARIES = libpicopter.a
libpicopter_a_SOURCES = BTLEServer.cpp \
						BTLEServer.h \
						CraftProperties.h \
						drone.cpp \
						drone.h \
						DroneState.cpp \
						DroneState.h \
						EigenTypes.h \
						EventLoop.cpp \
						EventLoop.h \
						PIDController.cpp \
						PIDController.h \
						Simulation.cpp \
						Simulation.h \
						Timer.cpp \
						Timer.h \
						VariadicTable.h \
						math/Integrators.cpp \
						math/Integrators.h \
						sensors/ADXL345.cpp \
						sensors/ADXL345.h \
						sensors/BMP280.cpp \
						sensors/BMP280.h \
						sensors/HMC5883.cpp \
						sensors/HMC5883.h \
						sensors/ITG3200.cpp \
						sensors/ITG3200.h \
						sensors/SensorUtils.cpp \
						sensors/SensorUtils.h
libpicopter_a_CPPFLAGS = -I$(srcdir)/../eigen
