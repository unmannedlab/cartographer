# cartographer
This is fork of https://github.com/cartographer-project/cartographer


## Installment

please follow the instruction in this link: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation

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