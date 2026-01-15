# SLAM afsluiten!!!
**Ga pas naar de volgende terminal nadat het huidige proces volledig is gestopt**

1. Sluit terminal **3** af met Ctrl+C
2. Sluit terminal **4** af met Ctrl+C
3. Sluit terminal **2** af met Ctrl+C
4. Sluit terminal **1** af met Ctrl+C

Met het commando ```ps -ef | grep ros``` kun je de achtergrondprocessen zien die nog draaien, dit zou er slechts 1 moeten zijn (ongeveer): `rens        3084    2050 50 13:39 pts/0    00:00:00 grep --color=auto ros`

Met het volgende commando kun je deze afsluiten: `sudo pkill -f "ros|nav2|slam|python3"`

Als je de robot wilt afsluiten gebruik: `sudo shutdown now`

# Kaart maken
### 1.1 Start SLAM
**Prerequisites:**
1. Log in op de robot 
2. Check of je de enige ben die met de robot is verbonden met `w`, er zou slechts 1 regel moeten zijn.
3. Check of er geen achtergrondprocessen draaien met `ps -ef | grep ros`, wederom zou er maar 1 regel moeten verschijnen, als dat niet het geval is sluit deze processen af met `sudo pkill -f "ros|nav2|slam|python3"`
4. Start de terminals op zoals hieronder staat beschreven
5. Sluit de terminals af **exact** zoals hierboven beschreven staat!
 
### Terminal 1
**Wacht met het uitvoeren van de volgende terminal totdat de huidige terminal gaan nieuwe regels meer uitvoert (bij terminal 3 is dat: `[lifecycle_manager-10] [INFO] [1767704635.139748514] [lifecycle_manager_navigation]: Creating bond timer...
`**

ssh rens@{IP-ADRES}


`ros2 launch linorobot2_bringup bringup.launch.py`

### Terminal 2
ssh rens@{IP-ADRES}


`ros2 launch teleop_twist_joy teleop-launch.py joy_config:=logitech`

### Terminal 3
ssh rens@{IP-ADRES}


`ros2 launch linorobot2_navigation slam.launch.py`

**BELANGRIJK: zorg ervoor dat deze stopt voordat je het volgende commando start!**

### Terminal 4

`ros2 launch linorobot2_viz slam.launch.py`

### 1.2 Beweeg de robot om een kaart te genereren
Beweeg de robot handmatig totdat het volledige werkgebied verkend is.
Je kunt ook in RViz de 2D Goal Pose-tool gebruiken om automatisch doelposities te laten bereiken tijdens het mappen.

### 1.3 Kaart opslaan
Ga naar de map-folder:
`cd /home/rens/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps/`

**Sla de kaart op:**

`ros2 run nav2_map_server map_saver_cli -f {MAP_NAME} --ros-args -p save_map_timeout:=10000.0`

# 2. Map laden voor autonome navigatie
## Robot:

**Terminal 1**

`ros2 launch linorobot2_bringup bringup.launch.py`

**Terminal 2**

`ros2 launch teleop_twist_joy teleop-launch.py joy_config:=logitech`

**Terminal 3**

`ros2 launch linorobot2_navigation navigation.launch.py     map:=/home/rens/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps/{MAP_NAME}.yaml     rviz:=false`


## Lokaal:

**Terminal 4**

`ros2 launch linorobot2_viz navigation.launch.py`

(Klik vervolgens in SLAM op 2D goal pose estimate)

# Verbindingscontrole (Raspberry Pi ↔ Teensy ↔ IMU)
**Bekijk alle actieve topics:**

`ros2 topic list`

**IMU-data controleren:**

`ros2 topic echo /imu/data`

# Probleem: ‘serial port not found’
Soms helpt het om het systeem volledig af te sluiten:

`sudo shutdown -h now`

Daarna fysiek de USB-kabel los en weer vast maken:

<img width="668" height="891" alt="image" src="https://github.com/user-attachments/assets/cfa38624-d747-4fdd-bab4-156a11b1b5e1" />


# SLAM ISSUE - Robot en LiDAR : 

LiDAR en robot visual zijn omgedraaid. Je staat voor de robot en de visuals zeggen dat je erachter staat. 2wd_properties-urdf aanpassen: 

`rens@rens:~$ sudo nano ~/linorobot2_ws/install/linorobot2_description/share/linorobot2_description/urdf/2wd_properties.urdf.xacro`

<img width="432" height="125" alt="image" src="https://github.com/user-attachments/assets/5c19f34d-ae92-4132-8302-04dbb6a66f0f" />

Aanpassen naar 180 graden (pi) in plaats van 0 bij rpy (0, 0, 0)!!

# Achtergrond processen afsluiten
Check wat er open staat:

`ps -ef | grep ros`

Sluit alle processen:

`sudo pkill -f "ros|nav2|slam|python3"`

Nu zou er nog 1 proces tussen moeten staan als je weer checkt met:

'ps -ef | grep ros'