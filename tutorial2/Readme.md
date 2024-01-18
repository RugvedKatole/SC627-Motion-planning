# Tutorial 2 ROS service (C++) and ROS service (python)
In the last tutorial, we learned about working with ROS workspace, packages, messages, topics, publishers and subscribers.

In this tutorial, we will be writing a ros service server in cpp and a ros service client in Python. The services are request-response ways of communicating and ros enables communication irrespective of programming language.

### Step 1 Creating a Custom service and message
Create a new package tutorial2 with dependencies std_msgs roscpp rospy

`$ catkin_create_pkg tutorial2 std_msgs rospy roscpp`

Writing a custom message with int64 Data Type called num.
Make a new Directory msg.

`$ mkdir msg`

`$ echo "int64 num > msg/Num.msg"`

Similarly, Creating a custom service with input as two integers and return the sum.
the custom service looks like this

```
int64 a
int64 b
---
int64 sum
```

Create a file name add_two_ints.srv in [package]/srv

```
$ mkdir srv
$ cd srv
$ touch AddTwoInts.srv
```

Copy and Paste the contents of [AddTwoInts](tutorial2/srv/AddTwoInts.srv) from the srv folder.

Editing Package.xml and adding dependencies.

Make sure the following lines are uncommented in package.xml
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Editing CMakelists.txt

Add the genmsg dependency to the find_package call which already exists in your CMakeLists.txt so that you can generate messages.

```
# Do not just add this to your CMakeLists.txt, modify the existing text to add genmsg before the closing parenthesis
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   genmsg
)
```

Edit the following block to add custom messages.
```add_message_files(
  FILES
  Num.msg
)
```
Uncomment the following lines in CMakelist.txt
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

A Similar setup to be followed for custom services.
```
add_service_files(
  FILES
  AddTwoInts.srv
)
```
checking if ros can see our services and messages

`$ rosmsg show [message Type]`

`$ rosmsg show tutorial2/Num`

`$ rossrv show [Service type]`

`$ rossrv show tutorial2/AddTwoInts.srv`

### Step 2 Writing a ROS service server in cpp

Create a file name `add_two_ints_server.cpp` inside [package]/src folder

`$ cd tutorial2 && touch src/add_two_ints_server.cpp`

Copy and paste the contents of src/add_two_ints_server.cpp into the file.

Add following lines to end of CMakeLists.txt 
```
include_directories(${catkin_INCLUDE_DIRS})
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
install(TARGETS add_two_ints_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

### Step 3 Writing a ROS service client in python

Create a file name `add_two_ints_client.py` inside [package]/scripts folder

`$ roscd tutorial2 && mkdir scripts`

`$ touch scripts/add_two_ints_client.py`

Copy and paste the contents of scripts/add_two_ints_client.py into the file.

### Step 4 Putting it all together

Navigating to Workspace folder and building our packages

`$ cd ~/catkin_ws && catkin_make`

`$ source devel/setup.bash`

Terminal 1: source the workspace

`$ roscore`

Terminal 2: source the workspace

`$ rosrun tutorial2 add_two_ints_server`

Terminal 3: source the workspace

`$ rosrun tutorial2 add_two_ints_client.py`
