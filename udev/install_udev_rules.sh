echo "Installing rules..."
sudo cp 59-cartpole.rules /etc/udev/rules.d/
echo "Restarting udev daemon..."
sudo /etc/init.d/udev restart
sudo udevadm control --reload-rules
echo "Good to go! (~˘▾˘)~"
