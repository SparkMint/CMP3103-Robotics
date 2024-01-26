# ros2-teaching-ws
A template repository for teaching robotics with ROS2

## Use case

You can use this repository to start developing your ROS2 modules. It provides a preconfigured [Development Container](https://containers.dev/) with ROS2 installed, and a VNC based light Desktop integrated directly.

## Usage

### Setup your environment

1. Make sure you have VSCode installed: https://code.visualstudio.com/download
2. Make sure you have the `Docker` and the `Dev Containers` extension in VSCode installed and working: https://code.visualstudio.com/docs/containers/overview and https://code.visualstudio.com/docs/devcontainers/containers
    * ensure docker is working, i.e. try `docker run --rm hello-world` and check it succeeds for your user
3. The docker image used to provide the Development Container is provided by the [L-CAS](https://lcas.lincoln.ac.uk) Container Registry. You must log in to use it. For simple read access, the username and password is public and is username `lcas`, password: `lincoln`. So, to log in do `docker login -u lcas -p lincoln lcas.lincoln.ac.uk` (you should only have to do this once, as the credentials should be cached unless your home directory is wiped).

### Open in VSCode

1. Open this repository in VSCode: https://code.visualstudio.com/docs/sourcecontrol/intro-to-git (or any other way you prefer)
2. VSCode should prompt you that there is a devcontainer configured and ask if you want to reopen in container. Re-open in the container

### Access the embedded lite Desktop

1. Click on the "Port" in VSCode, find the "novnc" port, right click on it to open the menu, and then choose either "Open in Browser" to open it outside of VSCode or "Preview in Editor" to have it open within VSCode:

   <img width="735" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2b0bdfa9-07ea-4238-a0b9-dd2dc8f4c111">

2. (recommended) Set the dekstop scaling by clicking on the settings cog and choose scaling mode "Remote Resizing" if it's not set

   <img width="292" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/2d9bc88e-7319-4723-968a-0aa08db026ef">

3. click on "Connect" and enter the password `vscode` when prompted:

   <img width="455" alt="image" src="https://github.com/LCAS/ros2-teaching-ws/assets/1153084/ddc224eb-5980-4d9a-994e-b05aa1e9fc1d">


