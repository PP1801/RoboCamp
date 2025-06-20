# RoboCamp

Programi RoboCamp sustava:

1.  Unutar test_dir nalaze se python programi za pokretanje sustava:
    -  robocamp_main_server.py   (robocamp_main_server_wip.py)
    -  robocamp_jetson_server.py (samo za simulaciju, pravi programi su na https://github.com/CRTA-Lab/robocamp_jetson_server)
    -  robocamp_drone_functions.py
    -  NMEA_parser.py
    -  haversine_calc.py
2.  startsitl.sh i startsocat.sh su također potrebni za pokretanje

4.  RC_marker_viz_1.cs  -  Plugin za markere u Mission Planner-u
5.  esp8266_gps_v0      -  Folder s konačnom verzijom Arduino programa prototipa narukvice


# INSTALACIJA OKRUŽENJA:

Originalna instalacija okružja je na ASUS (CRTA LAB1) računalu, a preko navedenih uputa je uspješno instalirano i isprobano okružje preko VMware-a na stolnom računalu.
U oba slučaja je korišten UBUNTU 20.04 6 (isprobano 20.06.2025.).

## INSTALACIJA: ArduPilot SITL i MAVProxy
Instalacija je trajala 15 - 30 min (VMware).

Pratiti upute na: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md

U .bashrc dodati:
<pre> export PATH=$PATH:$HOME/ardupilot/Tools/autotest 
 export PATH=/usr/lib/ccache:$PATH </pre>

## INSTALACIJA: Gazebo i ArduPilot plugin
Instalacija je trajala ~ 15 min (VMware).

Pratitit upute na: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md

Prije zadnjeg koraka (**Run Simulator**) u .bashrc dodati:
<pre> export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH </pre>
te osvježiti s
<pre> source ~/.bashrc. </pre>

Preporuka je pokrenuti sim_vehicle **BEZ** argumenta **-v**, kako bi se izbjegao ponovni build cijelog paketa.

## INSTALACIJA: ROS Noetic i Catkin workspace
Instalacija je trajala ~ 20 min (VMware).

Pratiti upute na: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md

Za odabir ROS istalacije najbolje je uzeti **Desktop-Full Install**, iako se može odabrati i običan **Desktop Install**.
U slučaju obične instalacije potrebno je ručno instalirati dependency pakete:
<pre>sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo apt-get install ros-noetic-tf2-eigen
sudo apt-get install ros-noetic-control-toolbox
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
rosdep install --from-paths src --ignore-src -r -y </pre>

Te na kraju (**5. Build instructions**)

<pre>cd ~/catkin_ws
catkin build
source devel/setup.bash</pre>

Ukoliko se iz nekog razloga nije samo exportalo, ručno upisati u .bashrc:
<pre>source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
GAZEBO_MODEL_PATH=/home/putanec/ardupilot_gazebo/models:/home/putanec/catkin_ws/src/iq_sim/models</pre>

**Svakako usporediti .bashrc na kraju svih instalacija s copy_.bashrc.txt koji je ovjde naveden!**



# INSTALACIJA PROGRAMA:

## INSTALACIJA: Pymavlink
Pratiti upute sa službene stranice: https://pypi.org/project/pymavlink/

## INSTALACIJA: Mission Planner
Pratiti upute s: https://ardupilot.org/planner/docs/mission-planner-installation.html,

odnosno: https://www.mono-project.com/download/stable/#download-lin.

Nakon unzippanja MissionPlanner-Latest, maknuti **-Latest** iz imena (rename) i prebaciti u Home direktorij.

# POSLJEDNJI DETALJI:
## Za Mission Planner
Iz foldera za_MP prebaciti **logo.png**, **logo2.png**, **logo.txt** i **BurntOrange.mpsystheme** u Home/MissionPlanner/.

**RC_marker_viz_1.cs** treba prebaciti u Home/MissionPlanner/plugins.

Mission Planner će automatski napraviti build na markeru, jedino će prvi put to malo duže potrajati.

Nakon što se jednom sve pokrenulo, u DATA izborniku se s Ctrl+P otvara prozor s popisom aktivnih Plugin-a.

Svi koji se ne koriste mogu se ugasiti kako bi se inače Mission Planner brže pokretao.

Custom tema za izgled Mission Planner-a se može naći u izborniku CONFIG, traka Planner, pod **Theme**.


Ukoliko Mission Planner zbog mono platforme ne prikazuje dobro fontove i grafike, potrebno je dodati **fonts.conf** u direktorij **.config/fontconfig/** (moguće je da ne postoji).
Također, potrebno je instalirati:
<pre>sudo apt install fonts-freefont-ttf fonts-liberation fonts-dejavu
sudo apt install libgdiplus libc6 libx11-dev
sudo fc-cache -f -v</pre>

Te izvesti:
<pre>sudo apt update
sudo apt upgrade mono-complete</pre>


## Za ArduPilot simulator
Datoteku **locations.txt** s custom početnim koordinatama simulacije, potrebno je preseliti u **.config/ardupilot/** folder.

Moguće je da **/ardupilot** ne postoji, pa ga treba napraviti.

Stablo s **.config/** se može vidjeti preko VS Code-a, pokretanjem <pre> . code </pre> u čistom terminalu.

### Osjetljivi dio!
Potrebno je pronaći skriptu **sim_vehicle.py** u folderu /ardupilot/Tools/autotest/ te ju otvoriti.

Preko Ctrl+F upisati **wsl2_host_ip_str** (trebali bi biti negdje između 800. i 900. linije!).

Ovdje u for petlji treba pronaći dio **ports = [nešto]** (npr. linija 885).

Ukoliko piše <pre>**ports = [14550 + 10 * i]**</pre> treba promjeniti u <pre>**ports = [14550 + 10 * i, 14551 + 10 * i]**</pre> ili npr. <pre>ports = [p + 10 * i for p in [14550, 14551]]</pre>

Spremiti i zatvoriti sim_vehicle.py. Ovaj dio će sada preko MavProxy-ja otvarati 2 MAVLink porta, kako bi tijekom simulacije imali poveznicu i s Jetson-om (14551) i Mission Planner-om (14550).


## Za skripte i hardware
Potrebno je instalirati:
<pre> sudo apt install net-tools
 sudo apt install socat</pre>

I promjeniti svojstva skripti, tako da ih možemo pokrenuti iz terminala:
<pre> chmod +x startsocat.sh startsitl.sh</pre>

Kada se priključe SiK telemetry radio prijemnici, te se vide kao npr. portovi **/ttyUSB0** i **/ttyUSB1**:
<pre>ls /dev/tty*</pre>
Treba im omogućiti pristup (čitanje/pisanje):
<pre>sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1</pre>
