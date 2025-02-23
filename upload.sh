#!/bin/bash

# Variables
SRC_DIR="src/"
FTP_USER="your_ftp_username"
FTP_PASS="your_ftp_password"
FTP_HOST="your_ftp_server_address"
FTP_DEST_DIR="/path/to/remote/directory"

rsync -avz --progress --rsh="sshpass -p $FTP_PASS ssh -o StrictHostKeyChecking=no" $SRC_DIR $FTP_USER@$FTP_HOST:$FTP_DEST_DIR