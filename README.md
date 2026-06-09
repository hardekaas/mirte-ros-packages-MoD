```bash
echo "deb [trusted=yes] https://github.com/hardekaas/mirte-ros-packages-MoD/raw/ros_mirte_humble_jammy_amd64/ ./" | sudo tee /etc/apt/sources.list.d/hardekaas_mirte-ros-packages-MoD.list
echo "yaml https://github.com/hardekaas/mirte-ros-packages-MoD/raw/ros_mirte_humble_jammy_amd64/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-hardekaas_mirte-ros-packages-MoD.list
```
