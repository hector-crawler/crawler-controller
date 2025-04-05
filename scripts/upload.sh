#!/bin/bash

source scripts/upload-env.sh
rsync -r . --exclude={.pixi,upload-env.sh,build,install,log,**node_modules,**__pycache__} -- $SSH_USERNAME@$SSH_HOSTADDRESS:$SAVE_PATH
