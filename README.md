# 설정
### intel D415
1. 카메라 설치
```
# Intel 공식 라이브러리인 librealsense2를 설치해야 합니다.
sudo apt update && sudo apt upgrade -y
sudo apt install git cmake build-essential -y
# Intel RealSense SDK 저장소를 추가합니다.
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6B0FC61
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main"
# 드라이버 설치:
sudo apt update
sudo apt install librealsense2-utils librealsense2-dev librealsense2-dbg -y
```

2. 연결확인
lsusb | grep Intels

3. ROS2 사용
```
# 패키지 설치
sudo apt install ros-$ROS_DISTRO-realsense2-camera
```
4. intel node 실행
```
ros2 launch realsense2_camera rs_launch.py
```
   
