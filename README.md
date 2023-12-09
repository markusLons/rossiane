# AutoRace 2023
A ROS2 metapackage that has necessary packages for AutoRace 2023 challenge.

## Included packages

* `robot_description` - holds the SDF description of the simulated robot, including sensors.

* `referee_console` - holds a referee node that is required for participants to run.

* `robot_bringup` - holds launch files, worlds and multiple configurations that serve as an example and as a required system for AutoRace to work.

## Usage

1. Запуск мира

    ```bash
    ros2 launch robot_bringup autorace_2023.launch.py
    ```

2. Запуск камеры

    ```bash
    ros2 launch autorace_camera extrinsic_camera_calibration.launch.py
    ```

3. Запуск светофора, и всего что связано с миссиями

    ```bash
    ros2 run referee_console mission_autorace_2023_referee
    ```

4. Изменение конфигураций камеры

    ```bash
    ros2 run rqt_reconfigure rqt_reconfigure
    ```

5. Запуск нашего пакета (пока отдельно)

    ```bash
    ros2 run autorace_core_rossiane path_tracking
    ```
