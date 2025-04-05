#!/bin/bash

root=$( (cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" ; cd ..) &> /dev/null && pwd )

# build crawler-web-ui
echo "Building crawler-web-ui..."
cd $root/crawler-web-ui
npm install
npm run build

# install crawler-web-ui into crawler-web-api
echo "Installing crawler-web-ui into crawler_web_api..."
rm -r $root/crawler/crawler/crawler_web_api/web-ui
mkdir $root/crawler/crawler/crawler_web_api/web-ui
cp -r $root/crawler-web-ui/build/client/* $root/crawler/crawler/crawler_web_api/web-ui
