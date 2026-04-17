# PMI_Client

PMI(Position Marking Indicator)용 Qt5 기반 TCP 클라이언트 데스크톱 애플리케이션입니다. CMake로 빌드하며 C++17과 Qt5 Widgets를 사용합니다.

## 요구 사항

- CMake 3.16 이상
- C++17을 지원하는 컴파일러 (GCC, Clang 등)
- Qt5 **Widgets** 모듈

### Ubuntu / Debian 예시

```bash
sudo apt update
sudo apt install build-essential cmake qtbase5-dev
```

다른 배포판에서는 패키지 이름이 다를 수 있습니다. Qt5 개발 패키지에 `Widgets`가 포함되어 있는지 확인하세요.

## 빌드

`PMI_Client` 디렉터리(`CMakeLists.txt`가 있는 곳)에서:

```bash
cd /path/to/PMI_Client
mkdir -p build
cd build
cmake ..
cmake --build .
```

- `cmake ..`에서 Qt를 찾지 못하면 Qt 설치 경로를 지정할 수 있습니다. 예: `cmake .. -DCMAKE_PREFIX_PATH=/opt/Qt/5.15.2/gcc_64`
- 빌드 산출물(실행 파일)은 `build` 디렉터리에 생성됩니다. 프로젝트 이름은 `CMakeLists.txt`의 `project(...)`와 동일하며, 현재 설정에서는 **`PMI_Client`**입니다.

## 실행

빌드 디렉터리에서:

```bash
./PMI_Client
```

GUI가 열리면 호스트·포트를 입력한 뒤 **연결**로 TCP 서버에 접속할 수 있습니다. 기본값은 `127.0.0.1:9000`입니다.

이전에 다른 프로젝트 이름으로 구성된 `build` 폴더가 있다면 실행 파일 이름이 다를 수 있습니다. 그 경우 `build`를 비우고 위 빌드 절차를 다시 수행하세요.

```bash
rm -rf build && mkdir build && cd build && cmake .. && cmake --build .
```
