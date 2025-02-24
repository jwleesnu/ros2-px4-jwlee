# QGroundControl 설치 (Ubuntu 22.04 환경)

Published by 이재우

Last update date : 2024-06-20  

<br>

### 1. QGroundControl 최신 배포판 설치

아래 사이트에서 QGroundControl의 최신 배포판의 QGroundControl.AppImage 파일을 다운받습니다. (2024-06-20 현재 ver 4.4.0)
https://github.com/mavlink/qgroundcontrol/releases

### 2. 필요한 패키지 설치

실행하는데 필요한 패키지를 다음과 같은 명령어로 설치합니다. 

```bash
sudo add-apt-repository universe
sudo apt install libfuse2
sudo apt install libsdl2-dev
sudo apt-get install qtbase5-dev
```

### 3. QGroundControl 실행

다운로드한 AppImage 파일이 있는 폴더(ex. Application)로 이동하여 QGroundControl 을 실행합니다. 

```bash
$ cd Application
$ ./QGroundControl.AppImage
```

### 4. 실행 확인

픽스호크를 연결하였을 때 다음과 같은 화면이 잘 나오는지 확인합니다. 

![image](https://github.com/jwleesnu/PX4-ROS2-setting/assets/173290153/949a0fa9-0dc2-409f-b47b-40fe20434873)

<br>

### [Troubleshooting]

<br>

__#1.__ QGroundControl 을 실행했을 때 다음과 같은 화면이 나오면 아래 명령어를 입력합니다. 

<img src="https://github.com/jwleesnu/PX4-ROS2-setting/assets/173290153/84946554-0dd4-4782-960c-5a7901c54489" width = "700" height="150"/>

 <br>

 <br>

```bash
sudo apt-get remove modemmanager
```

<br>

__#2.__ 다음과 같은 시리얼 포트 관련 에러 메시지가 나오면 아래 명령어를 입력합니다. 

<br>

에러 메시지 : permission denied '/dev/ttyACM0' 또는 Error,cannot bind to the specified serial port

```bash
sudo chmod 666 /dev/ttyACM0
```










