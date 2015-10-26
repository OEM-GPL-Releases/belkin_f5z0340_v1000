#!/bin/bash
set -x
cd /opt/cruisecontrol/projects/plugins-branch/plugins/branches/backfire_smart_corewifi/
svn up
sudo make veryclean
sudo make light V=99
scp -p /opt/cruisecontrol/projects/plugins-branch/plugins/branches/backfire_smart_corewifi/output/*.* root@10.20.90.137:/var/www/html/2.0-plugins-firmware-nightly-release/openwrt
sudo make clean
sudo make light-prod V=99
scp -p /opt/cruisecontrol/projects/plugins-branch/plugins/branches/backfire_smart_corewifi/output/*.* root@10.20.90.137:/var/www/html/2.0-plugins-firmware-nightly-release/openwrt

