#!/bin/bash

source scripts/upload-env.sh
rsync -r . --exclude={.pixi,upload-env.sh,build,install,log,**node_modules,**__pycache__} -- $FTP_USER@$FTP_HOST:$FTP_PATH