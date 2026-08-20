# fast_lio_localization2

## py文件赋权限

    cd /fast_lio_localization2-cd/fast_lio_localization
    chmod +x ./*.py

## 编译

    cd /fast_lio_localization2-cd/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


## 使用

### 1、mid360测试

    ros2 launch fast_lio_localization mid360_localization.launch.py \
        map:=/home/unitree/go2_patrol/map/test.pcd \
        rviz:=true


### 2、hesai雷达测试
    ros2 launch fast_lio_localization hesai_localization.launch.py \
        map:=/home/unitree/go2_patrol/map/test.pcd



## py依赖库检查
1、安装情况

    python3 -m pip show numpy
    python3 -m pip show open3d
    python3 -m pip show tf_transformations

2、安装版本

    python3 - <<'PY'
    import numpy
    print("version:", numpy.__version__)
    print("path:", numpy.__file__)
    PY



