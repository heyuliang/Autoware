Installation
============

Spec Recommendation
-------------------

* Generic x86_64
    * Intel Core i7 (preferred), Core i5, Atom
    * 16GB to 32GB of main memory
    * More than 30GB of SSD
    * NVIDIA GTX GeForce GPU (980M or higher performance)
* NVIDIA DRIVE
    * DRIVE PX2, DRIVE Xavier (on the way)
    * More than 30GB of SSD

Build
-----

There are three choices to build Autoware.
Using Autoware :doc:`Installation/DockerImage`, which provides built Autoware,
is strongly recommended.
In case that you do not want to or can not use Docker, please follow the
instruction provided by :doc:`Installation/SourceBuild` or :doc:`Installation/CrossBuild`.

.. toctree::
   :maxdepth: 1

   Installation/DockerImage
   Installation/SourceBuild
   Installation/CrossBuild

Setup
-----

Install DNN model on COCO dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DNN-based nodes require their pretrained models.
The nodes in Autoware are:

* `YOLO <https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/README.md>`_ 
* `SSD <https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/detection/vision_detector/packages/vision_ssd_detect/README.md>`_ 

.. todo::

    Delete link to GitHub and Import README.md in vision_darknet_detect and vision_ssd_detect.

Please follow README.md in each package to install the models if you need these nodes.

.. warning::

    Autoware itself is licensed under BSD 3-Clause "New" or "Revised" License.
    However, pre-trained models of DNN that the users of Autoware use are expected to
    be licensed under another license. For example, KITTI is licensed under CC BY-NC-SA,
    which do not allow commercial uses. **Please follow each license of the models you use.**
