# LMNet

## Requirements
* NVIDIA GPU
* Velodyne 64 Sensor or Rosbag with Velodyne 64 data (You can visit rosbag store https://rosbag.tier4.jp/)


## How to compile
1. Pull `cnn_lidar_detector` branch in your home directory. `cd ~ && git clone -b cnn_lidar_detector https://github.com/CPFL/Autoware.git`
1. Install the Caffe pre-requisites. http://caffe.berkeleyvision.org/installation.html#prerequisites
1. Add Unpooling Layer to your Caffe from the following link: https://github.com/BVLC/caffe/files/1029484/UnpoolingLayer.zip (Follow the instructions inside the zip file `link.txt` on how to add it properly.)
1. Compile using MAKE method Caffe according to the instructions given in http://caffe.berkeleyvision.org/installation.html#compilation
1. Execute `make distribute`.
1. Download LMNet pretrained model: http://ertl.jp/~amonrroy/lmnet/LMNetV2.caffemodel
1. Download prototxt pretrained model: http://ertl.jp/~amonrroy/lmnet/LMNetV2.prototxt
1. Run the node in a sourced terminal `rosrun cnn_lidar_detector cnn_lidar_detector_node _network_definition_file:=/PATH/TO/PROTOTXT _pretrained_model_file:=/PATH/TO/MODEL`.
1. Play a rosbag with **PointCloud Velodyne-64 data**.


## Subscriptions
PointCloud topic (XYZI), default: `/points_raw`.
To set a custom topic use `_points_node:=/YOURTOPIC`.

## Publications

|topic|type|
|-----|----|
|`/points_class`|RGB PointCloud|
|`/image_intensity`|Image |
|`/image_range`| Image |
|`/image_x`| Image |
|`/image_y`| Image |
|`/image_z`| Image |

## Citation

```
@inproceedings{lmnet2018,
  title={LMNet: Real-time Multiclass Object Detection on {CPU} using 3D LiDAR},
  author={Kazuki Minemura and
               Hengfui Liau and
               Abraham Monrroy and
               Shinpei Kato},
  booktitle={Intelligent Robot Systems (ACIRS), Asia-Pacific Conference on},
  year={2018},
  organization={IEEE}
}
```

## Paper
http://arxiv.org/abs/1805.04902
