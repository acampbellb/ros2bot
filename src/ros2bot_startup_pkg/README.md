# ros2bot startup package

The startup package is a mixed type package, meaning it contains both C++ and Python
node implementations.

## Package composition 

Create and configure an ROS2 package with C++ and Python nodes.

Prerequisite: 

Make sure to save your ROS workspace as a vscode workspace, from the file menu select save as
workspace, and save the workspace file in the root folder of your ROS workspace. This will ensure
that all include files w/in the workspace can be located.

1.  Create a C++ package

    ```
    $ cd ~/ros2bot_ws/src
    $ ros2 pkg create ros2bot_startup_pkg --build-type ament_cmake
    ```

2.  Open the package.xml file and fill in maintainer, description, and license info.

3.  Open the c_cpp_properties.json file in the .vcode folder, make sure it has the correct include paths.
    It should resemble this:

    ```
    {
        "configurations": [
            {
                "name": "Linux",
                "includePath": [
                    "${workspaceFolder}/**",
                    "/opt/ros/foxy/include/**"
                ],
                "defines": [],
                "compilerPath": "/usr/bin/gcc",
                "cStandard": "gnu17",
                "cppStandard": "gnu++14",
                "intelliSenseMode": "linux-gcc-x64"
            }
        ],
        "version": 4
    }
    ```

4.  Create a new folder in the root of the workspace w/ the same name as the package,
    and create an empty __init__.py file w/in it.

    You can create python modules to import w/in your python nodes in this folder.

5.  Create a scripts folder w/in the root of the workspace, and within this folder create
    a file for a python node. All python node files must start with the following line.

    ```
    #!/usr/bin/env python3
    ```

    Additionally, if you will be building w/ the symlink-install option, or want to launch
    the script directly you will need to make it executable w/ chmod +x scripts/*.py

    To import the sample_module, it should be referenced as,

    ```
    import <package name>.<module name>
    ```

6.  Configure the package.xml file.

    a)  Add python build tool w/:

        ```
        <buildtool_depend>ament_cmake_python</buildtool_depend>
        ```

    b)  Add dependencies w/:

        ```
        <depend>rclcpp</depend>
        <depend>rclpy</depend>
        ```

7.  Configure the CMakeLists.txt file.

    a)  Add the following find package lines:

        ```
        find_package(ament_cmake_python REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(rclpy REQUIRED)
        ```

    b)  After the find package lines, add these lines:

        ```
        # Include Cpp "include" directory
        include_directories(include)

        # Create Cpp executable
        add_executable(ros2bot_base_node src/ros2bot_base_node.cpp)
        ament_target_dependencies(ros2bot_base_node rclcpp)

        # Install Cpp executables
        install(TARGETS
          ros2bot_base_node
          DESTINATION lib/${PROJECT_NAME}
        )

        # Install Python modules
        ament_python_install_package(${PROJECT_NAME})

        # Install Python executables
        install(PROGRAMS
          scripts/ros2bot_driver_node.py
          DESTINATION lib/${PROJECT_NAME}
        )
        ```

9.  Add launch directory.

    Create a launch directory for launch files.

    ```
    $ cd ~/ros2bot_ws/src/ros2bot_startup/
    $ mkdir launch
    ```

    To install your launch files you’ll need to add this to your CMakeLists.txt:

    ```
    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )
    ```

    The launch/ folder will be copied and install into:
    
    ```
    ~/ros2bot_ws/install/ros2bot_startup_pkg/share/ros2bot_startup_pkg/launch/
    ```

10. Add config directory.

    Create a config directory for configuration files.

    ```
    $ cd ~/ros2bot_ws/src/ros2bot_startup_pkg/
    $ mkdir config
    ```

    To install the config your config files you will need to add this to CMakeLists.txt:

    ```
    install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
    )
    ```

    After the compilation you will find YAML files inside:
    
    ```
    ~/ros2bot_ws/install/ros2bot_startup_pkg/share/ros2bot_startup_pkg/config/
    ```
    
11. Compile and run the package.

    ```
    $ cd ~/ros2bot_ws
    $ colcon build --packages-select ros2bot_startup_pkg
    ```

    ```
    $ ros2 run ros2bot_startup_pkg <cpp node>
    $ ros2 run ros2bot_startup_pkg <py node>
    ```