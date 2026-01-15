# 1. Systeem Overzicht

Dit project implementeert een modulaire "Fleet Agent" op de Linorobot. De robot kan hierdoor op afstand aangestuurd worden via server-commando's.

- Input: Een externe server stuurt een locatie-naam naar /move_command.Daarnaast stuurt de server een liftknop naar /vision_command.

- Verwerking: De nav_goal_node ontvangt de locatie-naam en zoekt de coördinaten op in locations.yaml en stuurt de Nav2 stack aan. De camera ontvangt de /vision_command en rekent met computer vision de afstand uit tussen de marker die overeenkomt met het nummer van de liftknop.                   

- Uitvoering: De robot rijdt naar het doel. Navigatie stuurt updates naar /robot_status. Hierdoor weet de goederenklep dat het open moet. Daarnaast stuurt de camera-node de afstanden door naar /position_command. Hier luistert de arm_control_node naar om de waardes netjes te weergeven. Hiermee weet de gebruiker van de arm hoe ver de arm een bepaalde richting in moet om de liftknop te raken. 

- Feedback: De robot_status_node vangt de updates op, vertaalt ze naar een status (IDLE, MOVING, etc.) 

# 2. Software Componenten


<img width="1090" height="679" alt="image" src="https://github.com/user-attachments/assets/276b078c-e662-4a11-bf93-d5e310a29cc1" />

## A. Custom Interfaces (linorobot2_interfaces)

Bevat de definitie van de berichten.

    Message: msg/RobotStatus.msg

    Inhoud:

        string robot_id: Unieke naam van de robot.

        int8 status: Status code (0=UNKNOWN, 1=IDLE, 2=MOVING, 99=ERROR).

        string description: Leesbare tekst (bijv. "Heading to kitchen").

        string target_node: Huidig doel.

## B. Robot Status Node (robot_status_node)

Vertaalt ongestructureerde navigatie-tekst naar bruikbare data voor de Fleet Manager.

```
Subscribes: /robot_status (std_msgs/String) – tekststatus van NavGoalNode.

Parameters:

    - Geen specifieke ROS parameters; robot_id wordt afgeleid uit de ROS namespace.
```

Logica:
```
- Parseert ontvangen tekst en bepaalt status code (IDLE, MOVING, ERROR).

- Haalt doel-locatie uit tekst (bijv. “Heading to kitchen”) en zet target_node.

- Verstuurd altijd een RobotStatus bericht naar Fleet Manager.
```

Extra velden:
```
- last_visited_node: onthoudt de laatst bereikte locatie.
```

Voorbeeld output:
```
data: Couldn't reach Alternative_Medicine_Deliverypoint
---
data: 'Unknown location: Charging_DoDock'
---
data: Heading to Charging_Dock
---
data: Couldn't reach Charging_Dock
---
data: Heading to Emergency_Department
---
data: Arrived at Emergency_Department
---
```

## C. Nav Goal Node (nav_goal_node)

De "bestuurder" van de robot. Beheert de locatielijst en stuurt Nav2 aan.

```
Subscribes: /move_command (std_msgs/String) – commando van server of terminal.

Publishes: /robot_status (std_msgs/String) – tekststatus van de robot (Heading to / Arrived at).

Parameters / Configuratie:

    - location_names: lijst van locaties uit YAML.

    - Voor elke locatie: [X, Y, W] coördinaten.
```
Logica:

```
- Ontvangt een locatie-naam en controleert deze in de locatielijst.

- Stuurt een Nav2 action goal naar /navigate_to_pose.

- Publiceert status updates (Heading to / Arrived at / Couldn't reach) op /robot_status.

- Dubbele commando's naar dezelfde locatie worden genegeerd.
```

Voorbeeld output: 

```
[INFO] [1768394462.300464947] [nav_goal_node]: Loaded: General_Medicine_Deliverypoint -> [X:2.0, Y:0.0]
[INFO] [1768394462.301177285] [nav_goal_node]: Loaded: Alternative_Medicine_Deliverypoint -> [X:1.5, Y:0.5]
[INFO] [1768394462.303440196] [nav_goal_node]: Loaded: Emergency_Department -> [X:0.5, Y:0.0]
[INFO] [1768394462.304198123] [nav_goal_node]: Loaded: Charging_Dock -> [X:0.0, Y:0.0]
[INFO] [1768394462.304416583] [nav_goal_node]: Nav Node Ready. Loaded 4 locations from YAML.
[INFO] [1768394501.313130378] [nav_goal_node]: Received command: Go to 'Alternative_Medicine_Deliverypoint'
[INFO] [1768394501.314769497] [nav_goal_node]: Heading to Alternative_Medicine_Deliverypoint
[INFO] [1768394674.607117295] [nav_goal_node]: Received command: Go to 'Charging_DoDock'
[ERROR] [1768394674.607348132] [nav_goal_node]: Unknown location: Charging_DoDock
[INFO] [1768394696.788770787] [nav_goal_node]: Received command: Go to 'Charging_Dock'
[INFO] [1768394696.789457300] [nav_goal_node]: Heading to Charging_Dock
[INFO] [1768394890.082128914] [nav_goal_node]: Received command: Go to 'Emergency_Department'
[INFO] [1768394890.083219833] [nav_goal_node]: Heading to Emergency_Department
```

## D. Camera Detection Node (camera_detection_node)

Detecteert ArUco markers via de robotcamera en publiceert de gemeten positie van de marker op /position_command.

```
Subscribes: /vision_command (std_msgs/String) – geeft aan welke marker gevolgd moet worden.

Publishes: /position_command (std_msgs/String) – bevat marker ID, X/Y/Z posities en knopstatus.

Parameters / Configuratie:

    markerLength_ – fysieke grootte van de ArUco marker (standaard 0.044 m).

    cameraMatrix_ – camera-intrinsics.

    distCoeffs_ – lensdistortie parameters.
```

Logica:

    - Capture van de camera (V4L2, 1280x720).

    - Detecteert markers via OpenCV ArUco.

    - Berekent pose (x, y, z) van marker t.o.v. camera.

    - Als Z < 7.0 cm → knop wordt als ingedrukt gezien.

    - Publiceert de data als string in hetzelfde formaat dat Arm Control Node kan uitlezen.

    - Teken marker en assen in een GUI-venster.

Voorbeeld output van /position_command:

```
'=== Marker ID: 0 | x: 120.16 cm | y: 112.126 cm | z: 115.658 cm | xyDist: 164.349 cm | knop: NIET ingedrukt' 
---
```

## E. Arm Control Node (arm_control_node)

Interpreteren van markerdata en realtime feedback over liftknop positie.

```

Subscribes: /position_command (std_msgs/String) – berichten van Camera Detection Node of andere nodes met markerposities.

```

Logica:

    - Parseert string van /position_command, haalt x, y, z en knopstatus eruit.

    - Print de informatie in mensvriendelijk formaat:

        - X = rechts/links

        - Y = omhoog/omlaag

        - Z = vooruit/achteruit

    - Toont of de knop “INGEDRUKT” of “NIET ingedrukt” is.

Voorbeeld output:

    === Liftbutton: 0 === 0.70 cm rechts, 1.66 cm omhoog, 6.35 cm vooruit, knop: INGEDRUKT

## F. Start Info Node (start_info_node)
Ontvangt en parsed startinformatie bij opstart van de robot.

```
Subscribes: /start_info (std_msgs/String) – berichten van server bij robotstart.
```

Logica:
```
- Parseert ontvangen bericht voor “Liftbutton” en “DestinationPoint”.

- Print de parsed informatie voor debugging/logging.
```

## G. Servo Control Node (micro-ROS, ESP32)

De servo_control_node is een micro-ROS node die draait op een ESP32 en verantwoordelijk is voor het fysiek aansturen van een servo (bijv. een klep of mechanisme) op basis van ROS-commando’s en robotstatus.

Deze node vormt de brug tussen het ROS 2 netwerk en embedded hardware. Het is een micro-ROS node (ESP32, Arduino framework).

``` 
Subscribes: /move_command (std_msgs/String) - Wordt gebruikt om de servo naar een positie te sturen bij nieuwe navigatiecommando’s. Hierdoor gaat de klep van het goederenbakje dicht.

/robot_status (std_msgs/String) - Wordt gebruikt om te detecteren wanneer de robot zijn doel heeft bereikt. Hierdoor gaat de goederenklep open.
```

Functionaliteit:
```
- Stuurt een servo aan

- Gebruikt statusinformatie uit ROS om autonoom te reageren:

      - Bij ontvangst van een move command → servo sluit

      - Bij detectie van "Arrived" in /robot_status → servo opent

- Ondersteunt daarnaast lokale handmatige bediening via een fysieke drukknop (GPIO14)
```

Logica:
```
- De node analyseert inkomende string-berichten

- Bij het bereiken van een doel wordt de servo automatisch geopend

- De servo beweegt gecontroleerd naar zijn doelpositie

- Een fysieke knop kan de servo lokaal toggelen, onafhankelijk van ROS
```

# 3. Configuratie (Locaties)

Locaties worden niet hardcoded in C++, maar beheerd in een configuratiebestand. Je kunt de coordinaten verkrijgen door een map te laden, de robot naar een locatie te verplaatsen binnen deze map en in een nieuwe terminal het volgende commando uit te voeren: 

`ros2 topic echo /amcl_pose --0nce`

Dit commando versuurd de huidige locate en orientatie van de robot als volgt: 

    pose:
      pose:
        position:
          x: 3.45     <-- dit is de X-waarde
          y: -1.20    <-- dit is de Y-waarde
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.12
          w: 0.99     <-- dit is de W-waarde

Deze waardes
    Bestand: ~/linorobot2_ws/src/nav_goal_node/config/locations.yaml

    Structuur:
    YAML

    /nav_goal_node:
      ros__parameters:
        location_names: ["dock11", "pickup11", "dropoff11]

        # Formaat: [X, Y, W] (Positie X, Positie Y, Oriëntatie)
        dock11: [0.0, 0.0, 1.0]
        pickup11: [1.5, 0.5, 1.0]
        dropoff11: [3.45, -1.2, 0.99]

# 4. Gebruik van Standaard ROS 2 Launchfiles en Nodes

Naast de zelf ontwikkelde ROS-nodes maakt het project gebruik van bestaande Linorobot2- en ROS 2 launchfiles, die de basisfunctionaliteit van de robot verzorgen.

## 4.1 Robot Bringup
```
ros2 launch linorobot2_bringup bringup.launch.py
```

Deze launchfile start de basisfunctionaliteit van de robot, waaronder:

- motorcontrollers

- odometrie

- TF-tree (base_link, odom, map)

- sensordata (LIDAR, IMU)


Deze nodes vormen de fysieke interface tussen ROS en de robothardware.

## 4.2 Handmatige Besturing (Teleoperation)
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=logitech
```

Deze standaard ROS-node maakt handmatige besturing van de robot mogelijk via een joystick.
Dit wordt gebruikt voor hardwaretesten en het maken van een map. 

## 4.3 SLAM – Kaartgeneratie
```
ros2 launch linorobot2_navigation slam.launch.py
ros2 launch linorobot2_viz slam.launch.py
```

Deze launchfiles starten de SLAM-functionaliteit voor:

- het opbouwen van een kaart van de omgeving

- real-time lokalisatie

- visuele debugging in RViz

De gegenereerde kaart wordt later gebruikt voor autonome navigatie.

## 4.4 Autonome Navigatie met Kaart
```
ros2 launch linorobot2_navigation navigation.launch.py \
map:=/home/rens/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps/{MAP_NAME}.yaml \
rviz:=false
```

Deze launchfile start de volledige Nav2 stack met een vooraf gegenereerde kaart.

# 5. Topic Remapping en Namespaces

In de huidige implementatie is geen expliciete topic remapping toegepast.De standaard topicstructuur van ROS 2, Linorobot2 en Nav2 is aangehouden om compatibiliteit en stabiliteit te behouden.

Voor ondersteuning van meerdere robots wordt gebruikgemaakt van ROS namespaces, waardoor dezelfde node-structuur per robot hergebruikt kan worden zonder code-aanpassingen.

Voorbeeld:
```
/robot11/move_command	
/robot12/move_command	
/robot13/move_command	
```