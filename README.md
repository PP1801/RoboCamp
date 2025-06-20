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
te osvježiti s source ~/.bashrc
Preporuka je pokrenuti sim_vehicle **BEZ** argumenta **-v**, kako bi se izbjegao ponovni build cijelog paketa.

