AUTOMAKE_OPTIONS = subdir-objects
noinst_LIBRARIES = libpicopter.a
libpicopter_a_SOURCES = BTLEServer.cpp \
						BTLEServer.h \
						CraftProperties.h \
						drone.cpp \
						drone.h \
						dronecontrol.cpp \
						dronecontrol.h \
						DroneState.cpp \
						DroneState.h \
						EigenTypes.h \
						EventLoop.cpp \
						EventLoop.h \
						Simulation.cpp \
						Simulation.h \
						Timer.cpp \
						Timer.h \
						VariadicTable.h \
						ble/bleserver.cpp \
						ble/bleserver.h \
						math/Integrators.cpp \
						math/Integrators.h \
						math/PIDController.cpp \
						math/PIDController.h \
						math/utils.cpp \
						math/utils.h \
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
libpicopter_a_CPPFLAGS = -I$(srcdir)/../eigen -I$(srcdir)/../ggk/include $(GLIB_CFLAGS) $(GIO_CFLAGS) $(GOBJECT_CFLAGS)
