#!/bin/bash

# --new: create a new upload-env.sh file
if [[ $1 == "--new" ]]; then
  read -p "Enter environment name, or leave empty: " env_name
  read -p "Enter SSH username (pi): " SSH_USERNAME_input; SSH_USERNAME=${SSH_USERNAME_input:-pi}
  read -p "Enter SSH host address (e.g. 192.169.0.xxx): " SSH_HOSTADDRESS
  read -p "Enter SSH port (22): " SSH_PORT_input && SSH_PORT=${SSH_PORT_input:-22}
  read -p "Enter upload path (e.g. /home/pi/crawler): " UPLOAD_PATH_input; UPLOAD_PATH=${UPLOAD_PATH_input:-/home/pi/crawler}

  if [[ -z $env_name ]]; then file=scripts/upload-env.sh; else file=scripts/upload-env.$env_name.sh; fi
  if [[ -f $file ]]; then
    read -p "Environment $file already exists. Overwrite? [y/N] " overwrite
    if [[ $overwrite != "y" && $overwrite != "Y" ]]; then exit 1; fi
    rm $file
  fi
  echo -e "#!/bin/bash\nSSH_USERNAME=$SSH_USERNAME\nSSH_HOSTADDRESS=$SSH_HOSTADDRESS\nSSH_PORT=$SSH_PORT\nUPLOAD_PATH=$UPLOAD_PATH" >> $file
  echo "Environment created. Upload to it via: pixi run upload $env_name"
  exit 0
fi

# determine upload-env.sh and source it
if [[ ! -z $1 && $1 != "--build-web" ]] ; then
  source scripts/upload-env.$1.sh
else
  source scripts/upload-env.sh
fi

# check if upload-env.sh is configured
if [[ -z $SSH_HOSTADDRESS ]]; then
  echo "Please create an environment (pixi run upload --new) and specify it (pixi run upload <environment>)"
  exit 1
fi

# --build-web: optionally build crawler-web-ui before uploading
if [[ $1 == "--build-web" || $2 == "--build-web" ]]; then
  pixi run build-web
fi

# upload source files
rsync -r . --exclude={.git,.pixi,upload-env.sh,build,install,log,**node_modules,**__pycache__,.jj,.mypy_cache} \
    -v -e "ssh -p $SSH_PORT" -- $SSH_USERNAME@$SSH_HOSTADDRESS:$UPLOAD_PATH 
