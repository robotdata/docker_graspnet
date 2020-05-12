# docker_graspnet

A docker image of the GraspNet method.


## GraspNet
This docker image contain the code and the running environment of https://github.com/jsll/pytorch_6dof-graspnet
The code is in the folder of /pytorch_implementation

* Install docker and nvidia-docker on the host computer.
* Run `xhost +local:root` on the host computer to enable using GUI with docker.
  http://wiki.ros.org/docker/Tutorials/GUI
* `docker pull registry.gitlab.com/haiandaidi/docker_graspnet:2020_05_05`
* `docker run --gpus all -it --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -w /pytorch_implementation/pytorch_6dof-graspne registry.gitlab.com/teamhai/docker_collection /bin/bash`
* To run the demo:
  `python3 -m demo.main`
* Per default, the demo script runs the GAN sampler with sampling based refinement. To use the VAE sampler and/or gradient refinement run:
  `python3 -m demo.main --grasp_sampler_folder checkpoints/vae_pretrained/ --refinement_method gradient`
* The input data is in the folder of `demo/data/`
  Closing the opened window will let the algorithm go to process next input.
  To check the generated grasp information, read `demo/main.py`