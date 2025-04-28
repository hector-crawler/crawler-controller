# crawler-controller

The code for a crawling robot that learns to crawl by itself.

The robot is built using a Raspberry Pi 4, our code uses the
[Robot Operating System](https://ros.org/) (ROS) for interfacing
with the hardware.

### Repo overview

The repo consists of a ROS package (located in `./crawler`) which is
written in Python, as well as a web application in Typescript and [React](https://react.dev/)
(in `./crawler-web-ui`). The `./scripts` folder contains miscellaneous
files for building the various projects. We use [Pixi](https://pixi.sh/)
as our build tool as well as for package management.

The ROS package consists of a [Flask](https://palletsprojects.com/projects/flask/) server
and various ROS publishers/subscribers. The Flask server routes the calls and the WebSocket
connections that are made to the correct ROS commands.

The web app is built into a static HTML page that gets copied into the ROS package
as part of the build process and is served by the Flask server at runtime.

## Usage

### Setup

[Install Pixi](https://pixi.sh/latest/advanced/installation/)
on both the Raspberry Pi and your local development machine (It's essentially just
`curl -fsSL https://pixi.sh/install.sh | sh`).

You need to manually install the GPIO library [lgpio](https://abyz.me.uk/lg/download.html) globally on the Pi according to the instructions linked above, so the [rpi-lgpio](https://rpi-lgpio.readthedocs.io/) library can be installed.

Your Raspberry Pi needs to be accessible via SSH.

On your local development machine, run `pixi run upload --new` to configure your connection. 

### Build and run

Move the source files to the Pi (either by cloning the Git repository on the Pi or by running `pixi run upload` on your local machine).

Inside the source code folder on the Pi, run the following commands:

- `pixi install` to install required dependencies
- `pixi run build-web` to build the web app 
- `pixi run build-ros` to build the ROS package
- `pixi run launch` to launch all ROS nodes

Now you can access the web interface on port 5000 of the Raspberry Pi.

(Except for the final launch command, all other commands should also work
on the development machine.)
