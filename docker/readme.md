# コンテナ立ち上げ後作業・ビルド
cd /root/catkin_ws/src/
git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git
cd iai_kinect2
rosdep install -r --from-paths .

cd /root/catkin_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release -Dfreenect2_DIR=~/freenect2/lib/cmake/freenect2 -DCMAKE_CXX_STANDARD=14
source devel/setup.bash

# 動作テスト
roslaunch kinect2_bridge kinect2_bridge.launch
rosrun rviz rviz #color画像を表示する