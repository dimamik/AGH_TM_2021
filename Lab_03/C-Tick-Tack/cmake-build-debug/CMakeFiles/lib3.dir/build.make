# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/mnt/c/Users/Damian Cyper/Desktop/lcd"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/lib3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lib3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lib3.dir/flags.make

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o: CMakeFiles/lib3.dir/flags.make
CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o: ../stm32f7xx_hal_ltdc_ex.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o   -c "/mnt/c/Users/Damian Cyper/Desktop/lcd/stm32f7xx_hal_ltdc_ex.c"

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/mnt/c/Users/Damian Cyper/Desktop/lcd/stm32f7xx_hal_ltdc_ex.c" > CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.i

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/mnt/c/Users/Damian Cyper/Desktop/lcd/stm32f7xx_hal_ltdc_ex.c" -o CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.s

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.requires:

.PHONY : CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.requires

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.provides: CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.requires
	$(MAKE) -f CMakeFiles/lib3.dir/build.make CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.provides.build
.PHONY : CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.provides

CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.provides.build: CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o


# Object files for target lib3
lib3_OBJECTS = \
"CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o"

# External object files for target lib3
lib3_EXTERNAL_OBJECTS =

lib3: CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o
lib3: CMakeFiles/lib3.dir/build.make
lib3: CMakeFiles/lib3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable lib3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lib3.dir/build: lib3

.PHONY : CMakeFiles/lib3.dir/build

CMakeFiles/lib3.dir/requires: CMakeFiles/lib3.dir/stm32f7xx_hal_ltdc_ex.c.o.requires

.PHONY : CMakeFiles/lib3.dir/requires

CMakeFiles/lib3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lib3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lib3.dir/clean

CMakeFiles/lib3.dir/depend:
	cd "/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/Damian Cyper/Desktop/lcd" "/mnt/c/Users/Damian Cyper/Desktop/lcd" "/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug" "/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug" "/mnt/c/Users/Damian Cyper/Desktop/lcd/cmake-build-debug/CMakeFiles/lib3.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/lib3.dir/depend

