Bu Pr2 modelinin gazebo ortamında bir nesneyi tanıyarak o nesneye yönelen ardından nesneyi alarak başlangıç konumuna götürerek bırakan bir robot simülasyonu kodudur.


**Kendi ortamınızı kurarak robot reposunu bu ortama clone etmeniz gerekmektedir**


````mkdir -p ~/catkin_ws/src````
````cd ~/catkin_ws````
````catkin_make````
````source devel/setup.bash````

**Gazebo ortamında çalıştırılacak repoları terminale bu kodu yazarak kendi ortamınıza clone etmeniz gerekmektedir**


````cd ~/catkin_ws/src````
````git clone https://github.com/PR2/pr2_simulator.git````
````git clone https://github.com/PR2/pr2_common.git````

**Bağımlılıkları yükle**


````cd ~/catkin_ws````
````rosdep install --from-paths src --ignore-src -r -y````


**Terminal'de aşağıdaki komutu yazarak dünyayı başlatabilirsiniz**


````roslaunch pr2_gazebo pr2_empty_world.launch````

**Teleop Kontrolü: PR2 robotunu klavye ile kontrol etmek için:**


````roslaunch pr2_teleop teleop_keyboard.launch````

**Joystick kontrolü: Pr2 robotunu joysticklerle hareket ettirmek için**

````roslaunch pr2_teleop teleop_joystick.launch````


**Ana kameranın çıktısını görmek için rqt_image_view aracını çalıştırın** 


````rosrun rqt_image_view rqt_image_view````

**Projeyi başlatmak için**

````git clone https://github.com/Hilalbasibuyuk/pr2_robot.git````
````cd pr2_robot````
````python main.py````
