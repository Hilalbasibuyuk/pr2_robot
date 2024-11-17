Bu Pr2 modelinin gazebo ortamında bir nesneyi tanıyarak o nesneye yönelen ardından nesneyi alarak başlangıç konumuna götürerek bırakan bir robot simülasyonu kodudur.


**Kendi ortamınızı kurarak robot reposunu bu ortama clone etmeniz gerekmektedir**


mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash

**Gazebo ortamında çalıştırılacak repoları terminale bu kodu yazarak kendi ortamınıza clone etmeniz gerekmektedir**


cd ~/catkin_ws/src
git clone https://github.com/PR2/pr2_simulator.git
git clone https://github.com/PR2/pr2_common.git

**Bağımlılıkları yükle**


cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
