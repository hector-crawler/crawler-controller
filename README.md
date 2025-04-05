# crawler-controller

...


### Setup

On Raspberry Pi:
- make it accessible via SSH / FTP
- install [Pixi](https://pixi.sh): `curl -fsSL https://pixi.sh/install.sh | sh`
- install dependencies using Pixi: `pixi install`

In local development environment:
- create `upload-env.sh` inside `scripts` based on `upload-env.example.sh` and configure your connection

### Build and run

Move the source files to the Pi (either by cloning the Git repository on the Pi or by running `pixi run upload` on your local machine).

Inside the source code folder on the Pi, run the following commands:

- `pixi install` to install required dependencies
- `pixi run build-web` to build the web app (this also works on the development machine)
- `pixi run build` to build the ROS package
- `pixi run build-web` to build the web app 
- `pixi run build-ros` to build the ROS package
- `pixi run launch` to launch all ROS nodes

Now you can access the web interface on port 5000 of the Raspberry Pi.

(Except for the final launch command, all other commands should also work
on the development machine.)
