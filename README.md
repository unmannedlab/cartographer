# cartographer
This is fork of https://github.com/cartographer-project/cartographer


## Installment
Make sure your **CMAKE>3.6.0**
```
git clone https://github.com/unmannedlab/cartographer.git
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build stow
cd cartographer
src/cartographer/scripts/install_proto3.sh
sudo apt-get install libsuitesparse-dev
catkin_make_isolated --install --use-ninja
```


## Usage:

To perforam on-line slam:

run script
```
cart_config/run_3d_ouster
```


To perforam off-line slam:

run script
```
cart_config/run_3d_offline_ouster
```


visualize results:
```
cart_config/run_vis
```

generate map image and point cloud:
![bag_xray_xy_all](./example/_2020-02-15-14-05-50_0.bag_xray_xy_all.png)
```
cart_config/run_ass
```

## More configuration

Please refer to: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html