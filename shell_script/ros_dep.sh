echo "Install ros2-humble dependencies"
sudo apt update

echo -e "\n"
echo "✅ Install vision-msgs..."
sudo apt install ros-humble-vision-msgs

echo -e "\n"
echo "✅ Install diagnostic-updater..."
sudo apt install ros-humble-diagnostic-updater

echo -e "\n"
echo "✅ Install libpcap-dev..."
sudo apt install libpcap-dev -y

echo -e "\n"
echo "✅ Install pcl-ros..."
sudo apt install ros-humble-pcl-ros

echo -e "\n"
echo "✅ Install tf-transformations..."
sudo apt install ros-humble-tf-transformations -y
