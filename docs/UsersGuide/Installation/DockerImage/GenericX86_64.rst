Generic x86_64
==============

.. include:: DockerSetup.rst

NVIDIA Docker Setup
-------------------

You need to install Docker Plugin provided by NVIDIA in order to access NVIDIA GPUs from Docker Container.
If not already installed, Install CUDA drivers for your platform: Linux

1. Download the .deb file.

.. code-block:: shell

    $ wget https://github.com/NVIDIA/nvidia-docker/releases/download/v1.0.1/nvidia-docker_1.0.1-1_amd64.deb

2. Install the downloaded .deb file.

.. code-block:: shell

    $ sudo dpkg -i nvidia-docker_1.0.1-1_amd64.deb

3. Check if the nvidia-docker service exists.

.. code-block:: shell

    $ systemctl list-units --type=service | grep -i nvidia-docker

4. Check if the nvidia-docker service runs.

.. code-block:: shell

    $ sudo nvidia-docker run --rm nvidia/cuda nvidia-smi

.. note::

    If you run into the following error, try following these nvidia-docker `instructions <https://github.com/NVIDIA/nvidia-docker>`_.

        docker: Error response from daemon: OCI runtime create failed: container_linux.go:348: starting container process caused "exec: \"nvidia-smi\": executable file not found in $PATH": unknown.

Autoware Docker Setup
---------------------
