# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/cs184/cs184/projects/cs184-final-project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cs184/cs184/projects/cs184-final-project/build

# Include any dependencies generated for this target.
include CGL/deps/glew/CMakeFiles/glew.dir/depend.make

# Include the progress variables for this target.
include CGL/deps/glew/CMakeFiles/glew.dir/progress.make

# Include the compile flags for this target's objects.
include CGL/deps/glew/CMakeFiles/glew.dir/flags.make

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o: ../CGL/deps/glew/src/glew.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cs184/cs184/projects/cs184-final-project/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/glew.c.o   -c /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glew.c

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/glew.c.i"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glew.c > CMakeFiles/glew.dir/src/glew.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/glew.c.s"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glew.c -o CMakeFiles/glew.dir/src/glew.c.s

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.requires:
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.requires

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.provides: CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.requires
	$(MAKE) -f CGL/deps/glew/CMakeFiles/glew.dir/build.make CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.provides.build
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.provides

CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.provides.build: CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o: ../CGL/deps/glew/src/glewinfo.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cs184/cs184/projects/cs184-final-project/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/glewinfo.c.o   -c /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glewinfo.c

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/glewinfo.c.i"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glewinfo.c > CMakeFiles/glew.dir/src/glewinfo.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/glewinfo.c.s"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/glewinfo.c -o CMakeFiles/glew.dir/src/glewinfo.c.s

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.requires:
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.requires

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.provides: CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.requires
	$(MAKE) -f CGL/deps/glew/CMakeFiles/glew.dir/build.make CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.provides.build
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.provides

CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.provides.build: CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o: CGL/deps/glew/CMakeFiles/glew.dir/flags.make
CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o: ../CGL/deps/glew/src/visualinfo.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cs184/cs184/projects/cs184-final-project/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/glew.dir/src/visualinfo.c.o   -c /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/visualinfo.c

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/src/visualinfo.c.i"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/visualinfo.c > CMakeFiles/glew.dir/src/visualinfo.c.i

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/src/visualinfo.c.s"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew/src/visualinfo.c -o CMakeFiles/glew.dir/src/visualinfo.c.s

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.requires:
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.requires

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.provides: CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.requires
	$(MAKE) -f CGL/deps/glew/CMakeFiles/glew.dir/build.make CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.provides.build
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.provides

CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.provides.build: CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o

# Object files for target glew
glew_OBJECTS = \
"CMakeFiles/glew.dir/src/glew.c.o" \
"CMakeFiles/glew.dir/src/glewinfo.c.o" \
"CMakeFiles/glew.dir/src/visualinfo.c.o"

# External object files for target glew
glew_EXTERNAL_OBJECTS =

CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/build.make
CGL/deps/glew/libglew.a: CGL/deps/glew/CMakeFiles/glew.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C static library libglew.a"
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean_target.cmake
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glew.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CGL/deps/glew/CMakeFiles/glew.dir/build: CGL/deps/glew/libglew.a
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/build

CGL/deps/glew/CMakeFiles/glew.dir/requires: CGL/deps/glew/CMakeFiles/glew.dir/src/glew.c.o.requires
CGL/deps/glew/CMakeFiles/glew.dir/requires: CGL/deps/glew/CMakeFiles/glew.dir/src/glewinfo.c.o.requires
CGL/deps/glew/CMakeFiles/glew.dir/requires: CGL/deps/glew/CMakeFiles/glew.dir/src/visualinfo.c.o.requires
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/requires

CGL/deps/glew/CMakeFiles/glew.dir/clean:
	cd /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew && $(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean.cmake
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/clean

CGL/deps/glew/CMakeFiles/glew.dir/depend:
	cd /home/cs184/cs184/projects/cs184-final-project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs184/cs184/projects/cs184-final-project /home/cs184/cs184/projects/cs184-final-project/CGL/deps/glew /home/cs184/cs184/projects/cs184-final-project/build /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew /home/cs184/cs184/projects/cs184-final-project/build/CGL/deps/glew/CMakeFiles/glew.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CGL/deps/glew/CMakeFiles/glew.dir/depend

