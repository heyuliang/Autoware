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

Some nodes require additional setup.
Please follow the instructions if you want to use the nodes.

.. toctree::
    :maxdepth: 1

    Installation/SetupDNN
