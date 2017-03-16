SHELL := /bin/bash # Set shell
.SILENT: # Suppress output except warnings
# The files to compile
OBJS =  clipper.cpp Robot.cpp

#The compiler
CC = g++

# Compiler options
#-wall , -wextra -werror : for cleaner and safer code.
#-w suppresses all warnings (mutually exclusive with above flags)
# -g compiles with debug symbols
# -O2 best optimization without increasing executable size
COMPILER_FLAGS = -w -O2 -std=c++11 #-Wall -Wextra -Werror -Wno-sign-compare

# Compiler options for the debug executable
DBG_COMPILER_FLAGS = -g -Wall -Wno-unused-function -std=c++11

# Headers to include
INCLUDE = -I/usr/local/include/GeoLib -I/usr/local/include/MathGeoLib -I/usr/local/include/VVRScene 

# Libraries to link
LINKER_FLAGS = -lm -lGeoLib -lVVRScene -lMathGeoLib

# Executable name
OBJ_NAME = Robot


# Rules
all : $(OBJS)
	# Create executable
	$(CC) $(OBJS) $(COMPILER_FLAGS) $(LINKER_FLAGS) $(INCLUDE) -o $(OBJ_NAME)


debug : $(OBJS)
	
	# Create executable
	$(CC) $(OBJS) $(DBG_COMPILER_FLAGS) $(LINKER_FLAGS) $(INCLUDE) -o $(OBJ_NAME)

clean : $(OBJ_NAME)
	# Remove libraries
	$(MAKE) -C Libraries clean
	rm $(OBJ_NAME)

