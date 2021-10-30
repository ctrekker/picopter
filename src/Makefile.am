AUTOMAKE_OPTIONS = subdir-objects
noinst_LIBRARIES = libpicopter.a
libpicopter_a_SOURCES = BTLEServer.cpp \
						BTLEServer.h \
						CraftProperties.h \
						DroneState.cpp \
						DroneState.h \
						PIDController.cpp \
						PIDController.h \
						Simulation.cpp \
						Simulation.h \
						Timer.cpp \
						Timer.h \
						VariadicTable.h \
						sensors/ADXL345.cpp \
						sensors/ADXL345.h \
						sensors/BMP280.cpp \
						sensors/BMP280.h \
						sensors/HMC5883L.cpp \
						sensors/HMC5883L.h \
						sensors/ITG3200.cpp \
						sensors/ITG3200.h \
						sensors/SensorUtils.cpp \
						sensors/SensorUtils.h
libpicopter_a_CPPFLAGS = -I$(srcdir)/../eigen
