NVIDIA DRIVE
============

We provide Docker environments for the NVIDIA DRIVE platform with support from
NVIDIA Corporation. To access any of the following environments, you should have
signed their NDA/SLA to access NVIDIA DevZone and, in some case, should have been
authorized to access internal details of NVIDIA DriveWorks SDK.

DRIVE PX2
---------

If you are using the DRIVE PX2 platform, you can choose Docker with or without NVIDIA
DriveWorks. With NVIDIA DriveWorks, you may leverage NVIDIA self-driving capabilities,
such as line detection and object detection using DriveNet. Without DriveWorks, on the
other hand, all self-driving capabilities are covered by Autoware, thus for example
object detection is based on SSD or YOLO2. For more details, try it our yourself.

.. note::

    **DRIVE PX2 requires you to be a licensee of NVIDIA DRIVE and DevZone.**
    To complete the installation process introduced below, please contact Autoware
    Developers Group at autoware@googlegroups.com.

Docker Setup
^^^^^^^^^^^^

You first need to setup the Docker environment. You may not access docker.io with
the default configuration of Ubuntu 16.04, so try the following installation process.

.. code-block:: shell

    $ sudo apt-get install -y software-properties-common
    $ sudo apt-add-repository universe
    $ sudo apt-get update
    $ sudo apt-get install docker.io
    $ sudo apt-get update

Type the following commands.

.. code-block:: shell

    $ sudo docker info | grep "Docker Root Dir"

If you get the following output, you might continue with the process.

.. code-block:: text

    Docker Root Dir: /var/lib/docker

Autoware Setup
^^^^^^^^^^^^^^

.. todo::

    Insert document

DRIVE Xavier
------------

Coming soon.
