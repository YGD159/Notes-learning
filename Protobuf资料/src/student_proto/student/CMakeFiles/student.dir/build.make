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
CMAKE_SOURCE_DIR = /home/ygd/vsomeip/examples/student_proto

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ygd/vsomeip/examples/student_proto/student

# Include any dependencies generated for this target.
include CMakeFiles/student.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/student.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/student.dir/flags.make

CMakeFiles/student.dir/student.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/student.cpp.o: student.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ygd/vsomeip/examples/student_proto/student/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/student.dir/student.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/student.cpp.o -c /home/ygd/vsomeip/examples/student_proto/student/student.cpp

CMakeFiles/student.dir/student.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/student.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ygd/vsomeip/examples/student_proto/student/student.cpp > CMakeFiles/student.dir/student.cpp.i

CMakeFiles/student.dir/student.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/student.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ygd/vsomeip/examples/student_proto/student/student.cpp -o CMakeFiles/student.dir/student.cpp.s

CMakeFiles/student.dir/student.cpp.o.requires:

.PHONY : CMakeFiles/student.dir/student.cpp.o.requires

CMakeFiles/student.dir/student.cpp.o.provides: CMakeFiles/student.dir/student.cpp.o.requires
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/student.cpp.o.provides.build
.PHONY : CMakeFiles/student.dir/student.cpp.o.provides

CMakeFiles/student.dir/student.cpp.o.provides.build: CMakeFiles/student.dir/student.cpp.o


CMakeFiles/student.dir/student.pb.cc.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/student.pb.cc.o: student.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ygd/vsomeip/examples/student_proto/student/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/student.dir/student.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/student.pb.cc.o -c /home/ygd/vsomeip/examples/student_proto/student/student.pb.cc

CMakeFiles/student.dir/student.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/student.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ygd/vsomeip/examples/student_proto/student/student.pb.cc > CMakeFiles/student.dir/student.pb.cc.i

CMakeFiles/student.dir/student.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/student.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ygd/vsomeip/examples/student_proto/student/student.pb.cc -o CMakeFiles/student.dir/student.pb.cc.s

CMakeFiles/student.dir/student.pb.cc.o.requires:

.PHONY : CMakeFiles/student.dir/student.pb.cc.o.requires

CMakeFiles/student.dir/student.pb.cc.o.provides: CMakeFiles/student.dir/student.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/student.dir/build.make CMakeFiles/student.dir/student.pb.cc.o.provides.build
.PHONY : CMakeFiles/student.dir/student.pb.cc.o.provides

CMakeFiles/student.dir/student.pb.cc.o.provides.build: CMakeFiles/student.dir/student.pb.cc.o


# Object files for target student
student_OBJECTS = \
"CMakeFiles/student.dir/student.cpp.o" \
"CMakeFiles/student.dir/student.pb.cc.o"

# External object files for target student
student_EXTERNAL_OBJECTS =

../bin/student: CMakeFiles/student.dir/student.cpp.o
../bin/student: CMakeFiles/student.dir/student.pb.cc.o
../bin/student: CMakeFiles/student.dir/build.make
../bin/student: /usr/local/lib/libprotobuf.so
../bin/student: CMakeFiles/student.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ygd/vsomeip/examples/student_proto/student/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/student"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/student.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/student.dir/build: ../bin/student

.PHONY : CMakeFiles/student.dir/build

CMakeFiles/student.dir/requires: CMakeFiles/student.dir/student.cpp.o.requires
CMakeFiles/student.dir/requires: CMakeFiles/student.dir/student.pb.cc.o.requires

.PHONY : CMakeFiles/student.dir/requires

CMakeFiles/student.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/student.dir/cmake_clean.cmake
.PHONY : CMakeFiles/student.dir/clean

CMakeFiles/student.dir/depend:
	cd /home/ygd/vsomeip/examples/student_proto/student && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ygd/vsomeip/examples/student_proto /home/ygd/vsomeip/examples/student_proto /home/ygd/vsomeip/examples/student_proto/student /home/ygd/vsomeip/examples/student_proto/student /home/ygd/vsomeip/examples/student_proto/student/CMakeFiles/student.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/student.dir/depend

