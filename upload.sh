#!/bin/bash

source upload-env.sh
rsync src/ $FTP_USER@$FTP_HOST:$FTP_PATH