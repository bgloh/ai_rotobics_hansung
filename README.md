# 로보틱스 워크스페이스

덕키봇(Duckiebot) 제어를 위한 ROS 패키지들을 포함한 로보틱스 워크스페이스입니다.

## 프로젝트 구조

```
src/
├── my-proj1/
│   └── packages/
│       ├── led_control/          # LED 제어 패키지
│       ├── motor_control/        # 모터 제어 패키지  
│       ├── camera_control/       # 카메라 제어 패키지
│       └── my_first_package/     # 기본 pub/sub 예제
└── CLAUDE.md                     # 개발 가이드
```

## 패키지 설명

### 1. led_control 패키지
덕키봇의 LED 색상을 제어하는 패키지입니다.

**기능:**
- 단일 문자 명령을 통한 LED 색상 변경
- 지원 색상: GREEN(G), BLUE(B), RED(R), WHITE(W), OFF(O)
- rosservice를 통한 LED 패턴 설정

**실행 방법:**
```bash
rosrun led_control led_controller_node.py
```

### 2. motor_control 패키지  
덕키봇의 바퀴 모터를 제어하는 패키지입니다.

**기능:**
- WASD 키를 통한 방향 제어
- W: 전진, S: 후진, A: 좌회전, D: 우회전, X: 정지
- 멀티스레딩을 통한 빠른 응답 속도

**실행 방법:**
```bash
rosrun motor_control motor_controller_node.py
```

### 3. camera_control 패키지
덕키봇의 카메라 영상을 처리하는 패키지입니다.

**기능:**
- 그레이스케일 변환 및 엣지 검출
- 처리된 영상에 텍스트 오버레이 추가
- Docker 환경에서 동작하도록 최적화

**토픽:**
- 입력: `/{vehicle_name}/camera_node/image/compressed`
- 출력: 
  - `/{vehicle_name}/camera_node/image/processed/grayscaled/compressed`
  - `/{vehicle_name}/camera_node/image/processed/edgedetected/compressed`

**실행 방법:**
```bash
rosrun camera_control camera_controller_node.py
```

### 4. my_first_package 패키지
ROS의 기본 publisher/subscriber 패턴을 학습하기 위한 예제 패키지입니다.

**구성:**
- `my_publisher_node.py`: 메시지 발행 노드
- `my_subscriber_node.py`: 메시지 구독 노드

## ROS 패키지 생성 방법

Duckietown 가이드라인에 따른 catkin 패키지 생성 방법입니다.

### 1단계: 패키지 디렉토리 생성
```bash
mkdir -p ./packages/my_package
```

### 2단계: package.xml 파일 생성
```xml
<package>
  <name>my_package</name>
  <version>0.0.1</version>
  <description>
  Duckietown에서의 첫 번째 Catkin 패키지입니다.
  </description>
  <maintainer email="[your email]">YOUR_FULL_NAME</maintainer>
  <license>None</license>

  <buildtool_depend>catkin</buildtool_depend>
</package>
```

### 3단계: CMakeLists.txt 파일 생성
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
```

### 4단계: Python 스크립트 추가 (선택사항)
Python 노드를 추가하려면:

1. `scripts/` 디렉토리 생성:
```bash
mkdir -p ./packages/my_package/scripts
```

2. Python 스크립트 생성 후 실행 권한 부여:
```bash
chmod +x ./packages/my_package/scripts/my_node.py
```

3. CMakeLists.txt에 스크립트 추가:
```cmake
catkin_install_python(PROGRAMS
  scripts/my_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 빌드 및 실행

### 워크스페이스 빌드
```bash
cd /path/to/robotics_hansung/src
catkin_make
source devel/setup.bash
```

### 개별 노드 실행
```bash
# LED 컨트롤러
rosrun led_control led_controller_node.py

# 모터 컨트롤러
rosrun motor_control motor_controller_node.py

# 카메라 컨트롤러
rosrun camera_control camera_controller_node.py

# 퍼블리셰/서브스크라이버 예제
rosrun my_first_package my_publisher_node.py
rosrun my_first_package my_subscriber_node.py
```

## 개발 환경 설정

### 필요 사항
- ROS (Robot Operating System)
- Python 3
- OpenCV
- Duckietown 소프트웨어 스택

### 의존성 설치
각 패키지의 의존성은 해당 package.xml 파일에 정의되어 있습니다. 기본적으로 다음이 필요합니다:
- `rospy`: ROS Python 클라이언트 라이브러리
- `std_msgs`: 표준 메시지 타입
- `sensor_msgs`: 센서 데이터 메시지
- `cv_bridge`: OpenCV와 ROS 간 이미지 변환

## 주요 특징

1. **실시간 제어**: 모든 컨트롤러 노드는 실시간 인터랙티브 모드 지원
2. **Docker 호환**: 라즈베리파이 Docker 환경에서 동작하도록 설계
3. **모듈화 설계**: 각 기능별로 독립적인 패키지로 구성
4. **교육용 최적화**: 로보틱스 학습을 위한 명확한 구조와 주석

## 문제해결

### 일반적인 문제들
1. **권한 문제**: 스크립트 파일에 실행 권한이 있는지 확인
2. **환경 변수**: `VEHICLE_NAME` 환경 변수가 설정되어 있는지 확인  
3. **의존성**: 모든 ROS 의존성이 설치되어 있는지 확인

### 로그 확인
```bash
# ROS 로그 확인
roslog find [package_name]

# 노드 상태 확인  
rosnode list
rosnode info /node_name
```
