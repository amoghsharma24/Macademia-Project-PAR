# Important stuff for the project.

## 2.8   Saving the map for later use
From the navigation unit on the construct: 


To save the map you have created, you need to run an executable map_saver which runs a map_saver node from nav2_map_server.

IMPORTANT: call the node inside the directory where you want to save the map.

The command is as follows.


`cd ~/ros2_ws/src/cartographer_slam/config`

`ros2 run nav2_map_server map_saver_cli -f turtlebot_area`

Expected output:

    [INFO] [1668679453.925832478] [map_saver]:
            map_saver lifecycle node launched.
            Waiting on external lifecycle transitions to activate
            See https://design.ros2.org/articles/node_lifecycle.html for more information.
    [INFO] [1668679453.925942610] [map_saver]: Creating
    [INFO] [1668679453.926403154] [map_saver]: Saving map from 'map' topic to 'turtlebot_area' file
    [WARN] [1668679453.926451020] [map_saver]: Free threshold unspecified. Setting it to default value: 0.250000
    [WARN] [1668679453.926488575] [map_saver]: Occupied threshold unspecified. Setting it to default value: 0.650000
    [WARN] [map_io]: Image format unspecified. Setting it to: pgm
    [INFO] [map_io]: Received a 160 X 141 map @ 0.05 m/pix
    [INFO] [map_io]: Writing map occupancy data to turtlebot_area.pgm
    [INFO] [map_io]: Writing map metadata to turtlebot_area.yaml
    [INFO] [map_io]: Map saved
    [INFO] [1668679454.042853506] [map_saver]: Map saved successfully
    [INFO] [1668679454.042968106] [map_saver]: Destroying

IMPORTANT: Let me state again that the map will be saved at the location where you execute the command.

IMPORTANT: Do not close the Cartographer node before calling the map_saver. Otherwise, you lose the map you created.

The saving command will generate two files:

- tutlebot_area.pgm image file is the map as an occupancy grid image.
- turtlebot_area.yaml file contains details about the resolution of the map.

    YAML File of the Map.

    image: turtlebot_area.pgm
    mode: trinary
    resolution: 0.05
    origin: [-5.9, -5.22, 0]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.25

**image**: Name of the file containing the image of the generated map.

**resolution**: Resolution of the map (in meters/pixel).

**origin**: Coordinates of the lower-left pixel on the map. These coordinates are given in 2D (x, y, z). The third value indicates rotation. If there is no rotation, the value will be 0.

**occupied_thresh**: Pixels with a value greater than this value will be considered an occupied zone (marked as an obstacle).

**free_thresh**: Pixels with a value smaller than this will be considered a completely free zone.

**negate**: Inverts the colors of the map. By default, white means completely free, and black means completely occupied.

## Environment Package list
par-template==1.0.0
hazard-cartographer==1.0.0
aiil-rosbot-demo==1.0.0
aiil-gazebo==1.0.0
action-msgs==2.0.3
action-tutorials-interfaces==0.33.9
action-tutorials-py==0.33.9
actionlib-msgs==5.3.6
actuator-msgs==0.0.1
ament-cmake-test==2.5.5
ament-copyright==0.17.4
ament-cppcheck==0.17.4
ament-cpplint==0.17.4
ament-flake8==0.17.4
ament-index-python==1.8.2
ament-lint==0.17.4
ament-lint-cmake==0.17.4
ament-package==0.16.5
ament-pep257==0.17.4
ament-uncrustify==0.17.4
ament-xmllint==0.17.4
angles==1.16.1
bond==4.1.2
builtin-interfaces==2.0.3
camera-calibration==5.0.11
composition-interfaces==2.0.3
control-msgs==5.7.0
cv-bridge==4.1.0
demo-nodes-py==0.33.9
depthai-ros-driver==2.12.2
depthai-ros-msgs==2.12.2
diagnostic-msgs==5.3.6
diagnostic-updater==4.2.6
domain-coordinator==0.12.0
dwb-msgs==1.3.10
example-interfaces==0.12.0
examples-rclpy-executors==0.19.7
examples-rclpy-minimal-action-client==0.19.7
examples-rclpy-minimal-action-server==0.19.7
examples-rclpy-minimal-client==0.19.7
examples-rclpy-minimal-publisher==0.19.7
examples-rclpy-minimal-service==0.19.7
examples-rclpy-minimal-subscriber==0.19.7
ffmpeg-image-transport-msgs==1.0.2
find-object-2d==0.7.1
foxglove-msgs==3.2.3
generate-parameter-library-py==0.6.0
geographic-msgs==1.0.6
geometry-msgs==5.3.6
gps-msgs==2.1.1
grid-map-msgs==2.2.2
image-geometry==4.1.0
interactive-markers==2.5.5
joint-state-publisher==2.4.0
joint-state-publisher-gui==2.4.0
laser-geometry==2.7.2
launch==3.4.10
launch-ros==0.26.11
launch-testing==3.4.10
launch-testing-ros==0.26.11
launch-xml==3.4.10
launch-yaml==3.4.10
lifecycle-msgs==2.0.3
logging-demo==0.33.9
map-msgs==2.4.1
message-filters==4.11.10
nav2-common==1.3.10
nav2-msgs==1.3.10
nav2-simple-commander==1.0.0
nav-2d-msgs==1.3.10
nav-msgs==5.3.6
octomap-msgs==2.0.1
osrf-pycommon==2.1.7
pal-statistics==2.7.0
pal-statistics-msgs==2.7.0
pcl-msgs==1.0.0
pendulum-msgs==0.33.9
python-qt-binding==2.2.2
qt-dotgraph==2.7.5
qt-gui==2.7.5
qt-gui-cpp==2.7.5
qt-gui-py-common==2.7.5
quality-of-service-demo-py==0.33.9
rcl-interfaces==2.0.3
rclpy==7.1.9
rcutils==6.7.5
resource-retriever==3.4.4
rmw-dds-common==3.1.1
robot-localization==3.8.3
ros2action==0.32.8
ros2bag==0.26.9
ros2bag-mcap-cli==0.26.9
ros2bag-sqlite3-cli==0.26.9
ros2cli==0.32.8
ros2component==0.32.8
ros2doctor==0.32.8
ros2interface==0.32.8
ros2launch==0.26.11
ros2lifecycle==0.32.8
ros2multicast==0.32.8
ros2node==0.32.8
ros2param==0.32.8
ros2pkg==0.32.8
ros2plugin==5.4.4
ros2run==0.32.8
ros2service==0.32.8
ros2topic==0.32.8
ros-gz-bridge==1.0.18
ros-gz-interfaces==1.0.18
ros-gz-sim==1.0.18
rosapi==2.4.2
rosapi-msgs==2.4.2
rosbag2-interfaces==0.26.9
rosbag2-py==0.26.9
rosbridge-library==2.4.2
rosbridge-msgs==2.4.2
rosbridge-server==2.4.2
rosgraph-msgs==2.0.3
rosidl-adapter==4.6.7
rosidl-cli==4.6.7
rosidl-cmake==4.6.7
rosidl-generator-c==4.6.7
rosidl-generator-cpp==4.6.7
rosidl-generator-py==0.22.2
rosidl-generator-type-description==4.6.7
rosidl-parser==4.6.7
rosidl-pycommon==4.6.7
rosidl-runtime-py==0.13.1
rosidl-typesupport-c==3.2.2
rosidl-typesupport-cpp==3.2.2
rosidl-typesupport-fastrtps-c==3.6.3
rosidl-typesupport-fastrtps-cpp==3.6.3
rosidl-typesupport-introspection-c==4.6.7
rosidl-typesupport-introspection-cpp==4.6.7
rpyutils==0.4.2
rqt-action==2.2.1
rqt-bag==1.5.6
rqt-bag-plugins==1.5.6
rqt-console==2.2.2
rqt-graph==1.5.6
rqt-gui==1.6.3
rqt-gui-py==1.6.3
rqt-msg==1.5.2
rqt-plot==1.4.5
rqt-publisher==1.7.3
rqt-py-common==1.6.3
rqt-py-console==1.2.3
rqt-reconfigure==1.6.3
rqt-service-caller==1.2.2
rqt-shell==1.2.3
rqt-srv==1.2.3
rqt-tf-tree==1.0.5
rqt-topic==1.7.5
sensor-msgs==5.3.6
sensor-msgs-py==5.3.6
service-msgs==2.0.3
shape-msgs==5.3.6
simulation-interfaces==1.2.0
slam-toolbox==2.8.3
smclib==4.1.2
sros2==0.13.5
statistics-msgs==2.0.3
std-msgs==5.3.6
std-srvs==5.3.6
stereo-msgs==5.3.6
teleop-twist-keyboard==2.4.1
tf2-geometry-msgs==0.36.19
tf2-kdl==0.36.19
tf2-msgs==0.36.19
tf2-py==0.36.19
tf2-ros-py==0.36.19
tf2-sensor-msgs==0.36.19
tf2-tools==0.36.19
theora-image-transport==4.0.6
topic-monitor==0.33.9
trajectory-msgs==5.3.6
turtle-tf2-py==0.5.0
turtlesim==1.8.3
type-description-interfaces==2.0.3
unique-identifier-msgs==2.5.0
vision-msgs==4.1.1
visualization-msgs==5.3.6
xacro==2.1.1
babel==2.10.3
brlapi==0.8.5
brotli==1.1.0
deprecated==1.2.14
jinja2==3.1.2
markdown==3.5.2
markupsafe==2.1.5
pygobject==3.48.2
pyjwt==2.7.0
pynacl==1.5.0
pyqt5==5.15.10
pyqt5-sip==12.13.0
pyyaml==6.0.1
secretstorage==3.3.3
appdirs==1.4.4
argcomplete==3.1.4
attrs==23.2.0
bcrypt==3.2.2
beautifulsoup4==4.12.3
blinker==1.7.0
catkin-pkg-modules==1.1.0
cbor2==5.6.2
certifi==2023.11.17
chardet==5.2.0
click==8.1.6
cloud-init==25.3
colcon-argcomplete==0.3.3
colcon-bash==0.5.0
colcon-cd==0.2.1
colcon-cmake==0.2.29
colcon-common-extensions==0.3.0
colcon-core==0.20.1
colcon-defaults==0.2.9
colcon-devtools==0.3.0
colcon-installed-package-information==0.2.1
colcon-library-path==0.2.1
colcon-metadata==0.2.5
colcon-notification==0.3.0
colcon-output==0.2.13
colcon-override-check==0.0.1
colcon-package-information==0.4.0
colcon-package-selection==0.2.10
colcon-parallel-executor==0.3.0
colcon-pkg-config==0.1.0
colcon-powershell==0.4.0
colcon-python-setup-py==0.2.9
colcon-recursive-crawl==0.2.3
colcon-ros==0.5.0
colcon-test-result==0.3.8
colcon-zsh==0.5.0
colorama==0.4.6
command-not-found==0.3
configobj==5.0.8
contourpy==1.0.7
coverage==7.4.4
cryptography==41.0.7
cssselect==1.2.0
cupshelpers==1.0
cycler==0.11.0
dbus-python==1.3.2
decorator==5.1.1
defer==1.0.6
distlib==0.3.8
distro==1.9.0
distro-info==1.7+build1
dnspython==2.6.1
docutils==0.20.1
duplicity==2.1.4
empy==3.3.4
fasteners==0.18
flake8==7.0.0
flake8-builtins==2.1.0
flake8-comprehensions==3.14.0
flake8-docstrings==1.6.0
flake8-import-order==0.18.2
flake8-quotes==3.4.0
fonttools==4.46.0
freetype-py==2.4.0
fs==2.4.16
gpg==1.18.0
html5lib==1.1
httplib2==0.20.4
idna==3.6
importlib-metadata==4.12.0
iniconfig==1.1.1
jaraco.classes==3.2.1
jeepney==0.8.0
jsonpatch==1.32
jsonpointer==2.0
jsonschema==4.10.3
keyring==24.3.1
kiwisolver==0.0.0
language-selector==0.1
lark==1.1.9
launchpadlib==1.11.0
lazr.restfulclient==0.14.6
lazr.uri==1.0.6
libevdev==0.5
louis==3.29.0
lxml==5.2.1
lz4==4.0.2+dfsg
markdown-it-py==3.0.0
matplotlib==3.6.3
mccabe==0.7.0
mdurl==0.1.2
monotonic==1.6
more-itertools==10.2.0
mpi4py==3.1.5
mpmath==0.0.0
notify2==0.3
numpy==1.26.4
oauthlib==3.2.2
olefile==0.46
packaging==24.0
paramiko==2.12.0
pexpect==4.9.0
pillow==10.2.0
pluggy==1.4.0
protobuf==4.21.12
psutil==5.9.8
ptyprocess==0.7.0
pyopenssl==23.2.0
pycairo==1.25.1
pycodestyle==2.11.1
pycups==2.0.1
pydocstyle==6.3.0
pydot==1.4.2
pyflakes==3.2.0
pygments==2.17.2
pyparsing==3.1.1
pyrsistent==0.20.0
pyserial==3.5
pytest==7.4.4
pytest-cov==4.1.0
python-apt==2.7.7+ubuntu5.2
python-dateutil==2.8.2
python-debian==0.1.49+ubuntu2
pytz==2024.1
pyudev==0.24.0
pyxdg==0.28
reportlab==4.1.0
requests==2.31.0
rich==13.7.1
rlpycairo==0.3.0
roman==3.3
rosdistro-modules==1.0.1
rospkg-modules==1.6.1
scipy==1.11.4
semver==2.10.2
setuptools==68.1.2
six==1.16.0
snowballstemmer==2.2.0
soupsieve==2.5
ssh-import-id==5.11
sympy==1.12
systemd-python==235
toml==0.10.2
tornado==6.4
typeguard==4.1.5
ubuntu-drivers-common==0.0.0
ubuntu-pro-client==8001
ufolib2==0.16.0
ufw==0.36.2
ujson==5.9.0
unattended-upgrades==0.1
unicodedata2==15.1.0
urllib3==2.0.7
usb-creator==0.3.16
wadllib==1.3.6
webencodings==0.5.1
wrapt==1.15.0
xdg==5
xkit==0.0.0
zipp==1.0.0
