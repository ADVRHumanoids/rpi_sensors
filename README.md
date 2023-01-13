# rpi_sensors
Broadcast a few sensors over network from a RaspberryPi

### On the RaspberryPi
We can (optionally) install this package with `hhcm-forest`:
```
sudo pip install hhcm-forest
mkdir sensors_ws && cd sensors_ws
forest init
forest add-recipes https://github.com/advrhumanoids/multidof_recipes.git -t master
forest grow rpi_sensors -m rpi --clone-protocol https
```

To always have the workspace source: `echo $PWD/setup.bash >> ~/.bashrc`

To start udp camera streaming on user login (which should be automatic):
```
systemctl --user enable rpi_cam_sender.service 
sudo reboot
```


### On a ROS machine
We can (optionally) install this package with `hhcm-forest`:
```
sudo pip install hhcm-forest
mkdir sensors_ws && cd sensors_ws
forest init
forest add-recipes https://github.com/advrhumanoids/multidof_recipes.git -t master
forest grow rpi_sensors --clone-protocol https
```

To always have the workspace source: `echo $PWD/setup.bash >> ~/.bashrc`

To run the receiver node: `rosrun udp_cam_receiver udp_cam_receiver _remote_addr:=<RPI_IP_ADDR> _remote_port:=8080`
The video stream can then be seen with (e.g.) `rqt_image_view`
