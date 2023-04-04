# ros2_rust_template [![build-test](https://github.com/uhobeike/ros2_rust_template/actions/workflows/build-test.yaml/badge.svg)](https://github.com/uhobeike/ros2_rust_template/actions/workflows/build-test.yaml)

## Overview
ROS 2のRustパッケージのテンプレートです。（私のRust力が乏しいので参考程度にお願いします）

## Directory Configuration
```
.
├── LICENSE
├── README.md
├── launcher
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── multi_thread_pubsub.launch.py
│   │   └── parameter_multi_thread_pubsub.launch.py
│   └── package.xml
├── multi_thread_pubsub
│   ├── Cargo.lock
│   ├── Cargo.toml
│   ├── multi_thread_pub
│   │   ├── CMakeLists.txt
│   │   ├── Cargo.toml
│   │   ├── package.xml
│   │   └── src
│   │       └── main.rs
│   └── multi_thread_sub
│       ├── CMakeLists.txt
│       ├── Cargo.toml
│       ├── package.xml
│       └── src
│           └── main.rs
├── parameter
│   ├── Cargo.lock
│   ├── Cargo.toml
│   ├── parameter_multi_thread_pub
│   │   ├── CMakeLists.txt
│   │   ├── Cargo.toml
│   │   ├── package.xml
│   │   └── src
│   │       └── main.rs
│   └── parameter_multi_thread_sub
│       ├── CMakeLists.txt
│       ├── Cargo.toml
│       ├── package.xml
│       └── src
│           └── main.rs
└── pubsub
    ├── Cargo.lock
    ├── Cargo.toml
    ├── pub
    │   ├── CMakeLists.txt
    │   ├── Cargo.toml
    │   ├── package.xml
    │   └── src
    │       └── main.rs
    └── sub
        ├── CMakeLists.txt
        ├── Cargo.toml
        ├── package.xml
        └── src
            └── main.rs

17 directories, 36 files
```
## ROS 2 Rust Install（ROS 2インストール済み前提）
```
./ros2_rust_install_script.sh 
```

## Build / Install

```
mkdir catkin_ws/src -p
git clone https://github.com/uhobeike/ros2_rust_template.git catkin_ws/src/ros2_rust_template
cd catkin_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

###  Pub Sub
```
ros2 run sub sub 
ros2 run pub pub 
```

### Multi Thread Pub Sub Using Launch Action
```
ros2 launch launcher  multi_thread_pubsub.launch.py 
```

### Parameter Multi Thread Pub Sub Using Launch Action
```
ros2 launch launcher  parameter_multi_thread_pubsub.launch.py 
```

## [Rust code format](https://doc.rust-jp.rs/book-ja/appendix-04-useful-development-tools.html)
```
rustup component add rustfmt
cargo fmt
```

## Reference

### [https://tier4.github.io/safe_drive/](https://tier4.github.io/safe_drive/)
