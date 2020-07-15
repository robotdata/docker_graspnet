# docker_graspnet

A docker image of the GraspNet method.
ROS wrapper is added.

## GraspNet
This docker image contain the code and the running environment of https://github.com/jsll/pytorch_6dof-graspnet
The code is in the folder of graspnet_ws/src/graspnet/pytorch_6dof-graspnet

* Install docker and nvidia-docker on the host computer.
* Run `xhost +local:root` on the host computer to enable using GUI with docker.
  http://wiki.ros.org/docker/Tutorials/GUI
* `docker pull registry.gitlab.com/haiandaidi/docker_graspnet:2020_07_03_ros`
* `git pull https://gitlab.com/haiandaidi/docker_graspnet.git`
* `docker run --gpus all -it --network=host --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/graspnet_ws:/graspnet_ws:rw -w /graspnet_ws registry.gitlab.com/haiandaidi/docker_graspnet:2020_07_03_ros`
* To run the demo:
  `cd /graspnet_ws/src/graspnet/pytorch_6dof-graspnet`
  `python3 -m demo.main`
* Per default, the demo script runs the GAN sampler with sampling based refinement. To use the VAE sampler and/or gradient refinement run:
  `python3 -m demo.main --grasp_sampler_folder checkpoints/vae_pretrained/ --refinement_method gradient`
* The input data is in the folder of `demo/data/`
  Closing the opened window will let the algorithm go to process next input.
  To check the generated grasp information, read `demo/main.py`

## ROS wrapper
* `cd /graspnet_ws`
* `catkin_make`
* `source devel/setup.bash`
* `rosrun graspnet generate_grasp_poses.py`

* input topic: 'object_pc', PointCloud2 <br />
  This is pointcloud of the object to be grasped.
* output topic: 'generated_grasp_posearray', PoseArray <br />
  This include all the generated grasp poses.


These parameters can be changed in `script_dev/generate_grasp_poses.py`