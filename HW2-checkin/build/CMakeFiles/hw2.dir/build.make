# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.8.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.8.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build

# Include any dependencies generated for this target.
include CMakeFiles/hw2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hw2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hw2.dir/flags.make

CMakeFiles/hw2.dir/src/app.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/app.cpp.o: ../src/app.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hw2.dir/src/app.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/app.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/app.cpp

CMakeFiles/hw2.dir/src/app.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/app.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/app.cpp > CMakeFiles/hw2.dir/src/app.cpp.i

CMakeFiles/hw2.dir/src/app.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/app.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/app.cpp -o CMakeFiles/hw2.dir/src/app.cpp.s

CMakeFiles/hw2.dir/src/app.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/app.cpp.o.requires

CMakeFiles/hw2.dir/src/app.cpp.o.provides: CMakeFiles/hw2.dir/src/app.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/app.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/app.cpp.o.provides

CMakeFiles/hw2.dir/src/app.cpp.o.provides.build: CMakeFiles/hw2.dir/src/app.cpp.o


CMakeFiles/hw2.dir/src/camera.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/hw2.dir/src/camera.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/camera.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/camera.cpp

CMakeFiles/hw2.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/camera.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/camera.cpp > CMakeFiles/hw2.dir/src/camera.cpp.i

CMakeFiles/hw2.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/camera.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/camera.cpp -o CMakeFiles/hw2.dir/src/camera.cpp.s

CMakeFiles/hw2.dir/src/camera.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/camera.cpp.o.requires

CMakeFiles/hw2.dir/src/camera.cpp.o.provides: CMakeFiles/hw2.dir/src/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/camera.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/camera.cpp.o.provides

CMakeFiles/hw2.dir/src/camera.cpp.o.provides.build: CMakeFiles/hw2.dir/src/camera.cpp.o


CMakeFiles/hw2.dir/src/character.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/character.cpp.o: ../src/character.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/hw2.dir/src/character.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/character.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/character.cpp

CMakeFiles/hw2.dir/src/character.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/character.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/character.cpp > CMakeFiles/hw2.dir/src/character.cpp.i

CMakeFiles/hw2.dir/src/character.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/character.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/character.cpp -o CMakeFiles/hw2.dir/src/character.cpp.s

CMakeFiles/hw2.dir/src/character.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/character.cpp.o.requires

CMakeFiles/hw2.dir/src/character.cpp.o.provides: CMakeFiles/hw2.dir/src/character.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/character.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/character.cpp.o.provides

CMakeFiles/hw2.dir/src/character.cpp.o.provides.build: CMakeFiles/hw2.dir/src/character.cpp.o


CMakeFiles/hw2.dir/src/glfw.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/glfw.cpp.o: ../src/glfw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/hw2.dir/src/glfw.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/glfw.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/glfw.cpp

CMakeFiles/hw2.dir/src/glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/glfw.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/glfw.cpp > CMakeFiles/hw2.dir/src/glfw.cpp.i

CMakeFiles/hw2.dir/src/glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/glfw.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/glfw.cpp -o CMakeFiles/hw2.dir/src/glfw.cpp.s

CMakeFiles/hw2.dir/src/glfw.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/glfw.cpp.o.requires

CMakeFiles/hw2.dir/src/glfw.cpp.o.provides: CMakeFiles/hw2.dir/src/glfw.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/glfw.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/glfw.cpp.o.provides

CMakeFiles/hw2.dir/src/glfw.cpp.o.provides.build: CMakeFiles/hw2.dir/src/glfw.cpp.o


CMakeFiles/hw2.dir/src/item.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/item.cpp.o: ../src/item.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/hw2.dir/src/item.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/item.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item.cpp

CMakeFiles/hw2.dir/src/item.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/item.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item.cpp > CMakeFiles/hw2.dir/src/item.cpp.i

CMakeFiles/hw2.dir/src/item.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/item.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item.cpp -o CMakeFiles/hw2.dir/src/item.cpp.s

CMakeFiles/hw2.dir/src/item.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/item.cpp.o.requires

CMakeFiles/hw2.dir/src/item.cpp.o.provides: CMakeFiles/hw2.dir/src/item.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/item.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/item.cpp.o.provides

CMakeFiles/hw2.dir/src/item.cpp.o.provides.build: CMakeFiles/hw2.dir/src/item.cpp.o


CMakeFiles/hw2.dir/src/main.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/hw2.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/main.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/main.cpp

CMakeFiles/hw2.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/main.cpp > CMakeFiles/hw2.dir/src/main.cpp.i

CMakeFiles/hw2.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/main.cpp -o CMakeFiles/hw2.dir/src/main.cpp.s

CMakeFiles/hw2.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/main.cpp.o.requires

CMakeFiles/hw2.dir/src/main.cpp.o.provides: CMakeFiles/hw2.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/main.cpp.o.provides

CMakeFiles/hw2.dir/src/main.cpp.o.provides.build: CMakeFiles/hw2.dir/src/main.cpp.o


CMakeFiles/hw2.dir/src/menu.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/menu.cpp.o: ../src/menu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/hw2.dir/src/menu.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/menu.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/menu.cpp

CMakeFiles/hw2.dir/src/menu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/menu.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/menu.cpp > CMakeFiles/hw2.dir/src/menu.cpp.i

CMakeFiles/hw2.dir/src/menu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/menu.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/menu.cpp -o CMakeFiles/hw2.dir/src/menu.cpp.s

CMakeFiles/hw2.dir/src/menu.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/menu.cpp.o.requires

CMakeFiles/hw2.dir/src/menu.cpp.o.provides: CMakeFiles/hw2.dir/src/menu.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/menu.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/menu.cpp.o.provides

CMakeFiles/hw2.dir/src/menu.cpp.o.provides.build: CMakeFiles/hw2.dir/src/menu.cpp.o


CMakeFiles/hw2.dir/src/option.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/option.cpp.o: ../src/option.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/hw2.dir/src/option.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/option.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/option.cpp

CMakeFiles/hw2.dir/src/option.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/option.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/option.cpp > CMakeFiles/hw2.dir/src/option.cpp.i

CMakeFiles/hw2.dir/src/option.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/option.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/option.cpp -o CMakeFiles/hw2.dir/src/option.cpp.s

CMakeFiles/hw2.dir/src/option.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/option.cpp.o.requires

CMakeFiles/hw2.dir/src/option.cpp.o.provides: CMakeFiles/hw2.dir/src/option.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/option.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/option.cpp.o.provides

CMakeFiles/hw2.dir/src/option.cpp.o.provides.build: CMakeFiles/hw2.dir/src/option.cpp.o


CMakeFiles/hw2.dir/src/particle.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/particle.cpp.o: ../src/particle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/hw2.dir/src/particle.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/particle.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/particle.cpp

CMakeFiles/hw2.dir/src/particle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/particle.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/particle.cpp > CMakeFiles/hw2.dir/src/particle.cpp.i

CMakeFiles/hw2.dir/src/particle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/particle.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/particle.cpp -o CMakeFiles/hw2.dir/src/particle.cpp.s

CMakeFiles/hw2.dir/src/particle.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/particle.cpp.o.requires

CMakeFiles/hw2.dir/src/particle.cpp.o.provides: CMakeFiles/hw2.dir/src/particle.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/particle.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/particle.cpp.o.provides

CMakeFiles/hw2.dir/src/particle.cpp.o.provides.build: CMakeFiles/hw2.dir/src/particle.cpp.o


CMakeFiles/hw2.dir/src/scene.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/scene.cpp.o: ../src/scene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/hw2.dir/src/scene.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/scene.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene.cpp

CMakeFiles/hw2.dir/src/scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/scene.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene.cpp > CMakeFiles/hw2.dir/src/scene.cpp.i

CMakeFiles/hw2.dir/src/scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/scene.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene.cpp -o CMakeFiles/hw2.dir/src/scene.cpp.s

CMakeFiles/hw2.dir/src/scene.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/scene.cpp.o.requires

CMakeFiles/hw2.dir/src/scene.cpp.o.provides: CMakeFiles/hw2.dir/src/scene.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/scene.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/scene.cpp.o.provides

CMakeFiles/hw2.dir/src/scene.cpp.o.provides.build: CMakeFiles/hw2.dir/src/scene.cpp.o


CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o: ../src/shader/base_shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/base_shader.cpp

CMakeFiles/hw2.dir/src/shader/base_shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/shader/base_shader.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/base_shader.cpp > CMakeFiles/hw2.dir/src/shader/base_shader.cpp.i

CMakeFiles/hw2.dir/src/shader/base_shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/shader/base_shader.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/base_shader.cpp -o CMakeFiles/hw2.dir/src/shader/base_shader.cpp.s

CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.requires

CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.provides: CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.provides

CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.provides.build: CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o


CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o: ../src/shader/rectangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/rectangle.cpp

CMakeFiles/hw2.dir/src/shader/rectangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/shader/rectangle.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/rectangle.cpp > CMakeFiles/hw2.dir/src/shader/rectangle.cpp.i

CMakeFiles/hw2.dir/src/shader/rectangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/shader/rectangle.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/rectangle.cpp -o CMakeFiles/hw2.dir/src/shader/rectangle.cpp.s

CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.requires

CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.provides: CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.provides

CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.provides.build: CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o


CMakeFiles/hw2.dir/src/shader/skybox.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/shader/skybox.cpp.o: ../src/shader/skybox.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/hw2.dir/src/shader/skybox.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/shader/skybox.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/skybox.cpp

CMakeFiles/hw2.dir/src/shader/skybox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/shader/skybox.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/skybox.cpp > CMakeFiles/hw2.dir/src/shader/skybox.cpp.i

CMakeFiles/hw2.dir/src/shader/skybox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/shader/skybox.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/skybox.cpp -o CMakeFiles/hw2.dir/src/shader/skybox.cpp.s

CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.requires

CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.provides: CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.provides

CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.provides.build: CMakeFiles/hw2.dir/src/shader/skybox.cpp.o


CMakeFiles/hw2.dir/src/shader/text.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/shader/text.cpp.o: ../src/shader/text.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/hw2.dir/src/shader/text.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/shader/text.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/text.cpp

CMakeFiles/hw2.dir/src/shader/text.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/shader/text.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/text.cpp > CMakeFiles/hw2.dir/src/shader/text.cpp.i

CMakeFiles/hw2.dir/src/shader/text.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/shader/text.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/shader/text.cpp -o CMakeFiles/hw2.dir/src/shader/text.cpp.s

CMakeFiles/hw2.dir/src/shader/text.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/shader/text.cpp.o.requires

CMakeFiles/hw2.dir/src/shader/text.cpp.o.provides: CMakeFiles/hw2.dir/src/shader/text.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/shader/text.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/shader/text.cpp.o.provides

CMakeFiles/hw2.dir/src/shader/text.cpp.o.provides.build: CMakeFiles/hw2.dir/src/shader/text.cpp.o


CMakeFiles/hw2.dir/src/item/light_ball.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/item/light_ball.cpp.o: ../src/item/light_ball.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/hw2.dir/src/item/light_ball.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/item/light_ball.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item/light_ball.cpp

CMakeFiles/hw2.dir/src/item/light_ball.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/item/light_ball.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item/light_ball.cpp > CMakeFiles/hw2.dir/src/item/light_ball.cpp.i

CMakeFiles/hw2.dir/src/item/light_ball.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/item/light_ball.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/item/light_ball.cpp -o CMakeFiles/hw2.dir/src/item/light_ball.cpp.s

CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.requires

CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.provides: CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.provides

CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.provides.build: CMakeFiles/hw2.dir/src/item/light_ball.cpp.o


CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o: CMakeFiles/hw2.dir/flags.make
CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o: ../src/scene/string_scene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o -c /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene/string_scene.cpp

CMakeFiles/hw2.dir/src/scene/string_scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw2.dir/src/scene/string_scene.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene/string_scene.cpp > CMakeFiles/hw2.dir/src/scene/string_scene.cpp.i

CMakeFiles/hw2.dir/src/scene/string_scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw2.dir/src/scene/string_scene.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/src/scene/string_scene.cpp -o CMakeFiles/hw2.dir/src/scene/string_scene.cpp.s

CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.requires:

.PHONY : CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.requires

CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.provides: CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw2.dir/build.make CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.provides.build
.PHONY : CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.provides

CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.provides.build: CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o


# Object files for target hw2
hw2_OBJECTS = \
"CMakeFiles/hw2.dir/src/app.cpp.o" \
"CMakeFiles/hw2.dir/src/camera.cpp.o" \
"CMakeFiles/hw2.dir/src/character.cpp.o" \
"CMakeFiles/hw2.dir/src/glfw.cpp.o" \
"CMakeFiles/hw2.dir/src/item.cpp.o" \
"CMakeFiles/hw2.dir/src/main.cpp.o" \
"CMakeFiles/hw2.dir/src/menu.cpp.o" \
"CMakeFiles/hw2.dir/src/option.cpp.o" \
"CMakeFiles/hw2.dir/src/particle.cpp.o" \
"CMakeFiles/hw2.dir/src/scene.cpp.o" \
"CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o" \
"CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o" \
"CMakeFiles/hw2.dir/src/shader/skybox.cpp.o" \
"CMakeFiles/hw2.dir/src/shader/text.cpp.o" \
"CMakeFiles/hw2.dir/src/item/light_ball.cpp.o" \
"CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o"

# External object files for target hw2
hw2_EXTERNAL_OBJECTS =

hw2: CMakeFiles/hw2.dir/src/app.cpp.o
hw2: CMakeFiles/hw2.dir/src/camera.cpp.o
hw2: CMakeFiles/hw2.dir/src/character.cpp.o
hw2: CMakeFiles/hw2.dir/src/glfw.cpp.o
hw2: CMakeFiles/hw2.dir/src/item.cpp.o
hw2: CMakeFiles/hw2.dir/src/main.cpp.o
hw2: CMakeFiles/hw2.dir/src/menu.cpp.o
hw2: CMakeFiles/hw2.dir/src/option.cpp.o
hw2: CMakeFiles/hw2.dir/src/particle.cpp.o
hw2: CMakeFiles/hw2.dir/src/scene.cpp.o
hw2: CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o
hw2: CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o
hw2: CMakeFiles/hw2.dir/src/shader/skybox.cpp.o
hw2: CMakeFiles/hw2.dir/src/shader/text.cpp.o
hw2: CMakeFiles/hw2.dir/src/item/light_ball.cpp.o
hw2: CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o
hw2: CMakeFiles/hw2.dir/build.make
hw2: /Applications/MATLAB_R2015b.app/bin/maci64/libfreetype.dylib
hw2: /usr/local/lib/libglfw.3.2.dylib
hw2: /usr/local/lib/libGLEW.dylib
hw2: CMakeFiles/hw2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Linking CXX executable hw2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hw2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hw2.dir/build: hw2

.PHONY : CMakeFiles/hw2.dir/build

CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/app.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/camera.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/character.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/glfw.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/item.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/main.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/menu.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/option.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/particle.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/scene.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/shader/base_shader.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/shader/rectangle.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/shader/skybox.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/shader/text.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/item/light_ball.cpp.o.requires
CMakeFiles/hw2.dir/requires: CMakeFiles/hw2.dir/src/scene/string_scene.cpp.o.requires

.PHONY : CMakeFiles/hw2.dir/requires

CMakeFiles/hw2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hw2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hw2.dir/clean

CMakeFiles/hw2.dir/depend:
	cd /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build /Users/XP/Documents/Src/Animation-Planning-in-Games-HW/HW2-checkin/build/CMakeFiles/hw2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hw2.dir/depend

