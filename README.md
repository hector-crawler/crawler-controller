# crawler-controller

...


### Setup

On Raspberry Pi:
- make it accessible via SSH / FTP
- install [Pixi](https://pixi.sh): `curl -fsSL https://pixi.sh/install.sh | sh`
- install dependencies using Pixi: `pixi install`
- install [Node](https://nodejs.org) (via NVM): `curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.2/install.sh | bash`, `nvm install 20`

In local development environment:
- create `upload-env.sh` inside `scripts` based on `upload-env.example.sh` and configure your connection

### Build and run

- (locally) `pixi run upload` to upload source files to the Pi
- (on Pi) `pixi run build-web` (can also be done locally) and `pixi run build` to build the ROS package
- (on Pi) `pixi run launch` to launch ROS nodes
- access web interface on port 5000
