SDF модель робота на трех роликонесущих колесах и плагин управления им.

Симуляция реализуется средсвтами следующих программ:
ROS melodic (https://www.ros.org/)
Gazebo simulation 9 (http://gazebosim.org/)
CMake  3.10.2
g++ 7.5.0
qt 5 
Дополнительные утилиты ROS (описано ниже)

Рекомендуемая операционная система - Ubuntu 18.04

-------------------------------------------------------------------------
Установка программного обеспечения для операционной системы Ubuntu 18.04

(Подробно описана здесь http://wiki.ros.org/melodic/Installation/Ubuntu)

В командной строке
1) Добавляем репозиторий
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
2) Добавляем ключ для верификации
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
3) Устанавливаем ROS и Gazebo 9
sudo apt update
sudo apt install ros-melodic-desktop-full
4) Добавляем переменные окружения в автозапуск терминала
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
5) Устанавливаем зависимости для сборки модуля 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
6) Иницилизируем rosdep
sudo rosdep init
rosdep update

------------------------------------------------------------------------
Компиляция плагина

1) Создаем папку сборки
cd plugin
mkdir build 
cd build
2) Создаем cmake проект 
cmake ../
3) Компилируем его 
make -j 6
где 6 - кол-во потоков, используемых для сборки

Теперь в папке ../plugin/build/devel/lib/

libmain.so - откомпилированная библиотека / плагин управления роботом в gazebo

CartControlPlugin/client  исполняемый файл - клиент управления роботом 

-------------------------------------------------------------------------
Добавление модели робота в систему моделирования ros

1) В папку моделей gazebo добавить папку omni-wheel-robot
2) В файле model.sdf заменить пути к файлам на пути к файлам в своей системе с аналогичными названиями

-------------------------------------------------------------------------
Запуск симуляции

1)  В отдельном терминале запускаем мастер ноду  ros
roscore
В другом терминале
2) Синхронизируем gazebo и ros
rosparam set use_sim_time true
3) Запускаем gazebo
rosrun gazebo_ros gazebo
4) Добавляем робота в симуляцию из списка имеющихся объектов
5) Запускаем client и управляем роботом

Работа клиента описана в тексте дипломной работы, приложенному к диску
