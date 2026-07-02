#!/bin/bash
set -e

echo "=== ROS 2 Humble ve Gazebo Kurulum Scripti ==="

# 1. Locale Ayarları
echo "Locale ayarları yapılıyor..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. ROS 2 Depolarının Eklenmesi
echo "ROS 2 depoları ekleniyor..."
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. ROS 2 Humble Masaüstü Kurulumu
echo "ROS 2 Humble kuruluyor (Bu işlem birkaç dakika sürebilir, lütfen bekleyin)..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

# 4. Gazebo Kurulumu
echo "Gazebo ve ROS entegrasyon paketleri kuruluyor..."
sudo apt install -y ros-humble-gazebo-ros-pkgs gazebo11

# 5. Ortam Değişkenlerinin Eklenmesi
echo "Ortam değişkenleri .bashrc dosyasına ekleniyor..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "=== KURULUM BAŞARIYLA TAMAMLANDI ==="
echo "Lütfen yeni ayarların etkinleşmesi için terminali kapatıp açın veya 'source ~/.bashrc' komutunu çalıştırın."
