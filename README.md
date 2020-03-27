# try-a-bot
IEEE RAS CEMRA Project "Try-a-Bot" - docker version

For the VirtualBox machine see [https://sites.google.com/uji.es/try-a-bot/virtual-machine](https://sites.google.com/uji.es/try-a-bot/virtual-machine)

## Prerequisites
Docker [with NVIDIA support](https://github.com/NVIDIA/nvidia-docker).

## Setup
Clone this repository and build the docker image with:
````
./build.sh
````
Or pull a pre-built image with:
````
docker pull robinlab/try-a-bot:latest
````

## Run
Open a terminal and run:
````
./launch.sh
````

In a browser, open this link: [http://localhost:8888/lab/tree/index.ipynb](http://localhost:8888/lab/tree/index.ipynb)

In JupyterLab, launch the Webots simulator with the Pioneer world.

In JupyterLab, launch the controller notebook, and run the cells for moving and stopping the robot.
