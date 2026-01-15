# 1. Systeem Overzicht

Dit project implementeert een modulaire "Fleet Agent" op de Linorobot. De robot kan hierdoor op afstand aangestuurd worden via server-commando's en rapporteert gestructureerd zijn status terug.
Data Flow Architectuur

    Input: Een externe server (of terminal) stuurt een locatie-naam (bijv. "kitchen") naar /move_command.

    Verwerking: De nav_goal_node ontvangt dit, zoekt de coördinaten op in locations.yaml en stuurt de Nav2 stack aan.

    Uitvoering: De robot rijdt naar het doel. Navigatie stuurt updates naar /robot_status.

    Feedback: De robot_status_node vangt de updates op, vertaalt ze naar een status-code (IDLE, MOVING, etc.) en publiceert dit op /fleet_status.

# 2. Software Componenten

## A. Custom Interfaces (linorobot2_interfaces)

Bevat de definitie van de berichten.

    Message: msg/RobotStatus.msg

    Inhoud:

        string robot_id: Unieke naam van de robot.

        int8 status: Status code (0=UNKNOWN, 1=IDLE, 2=MOVING, 99=ERROR).

        string description: Leesbare tekst (bijv. "Heading to kitchen").

        string target_node: Huidig doel.

## B. Robot Status Node (robot_status_node)

Vertaalt ongestructureerde navigatie-tekst naar bruikbare data.

    Subscribes: /robot_status (std_msgs/String)

    Publishes: /fleet_status (linorobot2_interfaces/RobotStatus)

    Parameters: robot_id (default: "robot_1")

## C. Nav Goal Node (nav_goal_node)

De "bestuurder" van de robot. Beheert de locatielijst en stuurt Nav2 aan.

    Subscribes: /move_command (std_msgs/String)

    Action Client: /navigate_to_pose (Nav2)

    Config: Leest coördinaten uit een YAML bestand.

# 3. Configuratie (Locaties)

Locaties worden niet hardcoded in C++, maar beheerd in een configuratiebestand.

    Bestand: ~/linorobot2_ws/src/nav_goal_node/config/locations.yaml

    Structuur:
    YAML

    /nav_goal_node:
      ros__parameters:
        location_names: ["kitchen", "table", "elevator", "charging_dock"]

        # Formaat: [X, Y, W] (Positie X, Positie Y, Oriëntatie)
        kitchen: [2.0, 0.0, 1.0]
        table: [1.5, 0.5, 1.0]
        charging_dock: [0.0, 0.0, 1.0]

# 4. Installatie & Bouwen

Voer dit uit na wijzigingen in C++ code of package settings.

    cd ~/linorobot2_ws

    # 1. Oude build files opruimen (voorkomt vage errors)

    rm -rf build/ install/ log/

    # 2. Bouwen

    colcon build

    # 3. Omgeving verversen

    source install/setup.bash

# 5. User Guide: Opstarten

Het systeem vereist 4 terminals (of tabbladen).

### Terminal 1: Robot Drivers (Bringup)

Start de basis drivers van de robot (motoren, lidar, etc.).
Bash

`ros2 launch linorobot2_bringup bringup.launch.py`

### Terminal 2: Map & Navigatie

Laad de kaart en start de Nav2 stack.

`ros2 launch linorobot2_navigation navigation.launch.py     map:=/home/rens/base_robot_package/src/linorobot2/linorobot2_navigation/maps/{MAP_NAME}.yaml     rviz:=false`

### Terminal 3: Status Node

Start de vertaler die luistert naar de robot status.

`ros2 run robot_status_node robot_status_node`

### Terminal 4: Navigatie Manager

`ros2 run nav_goal_node nav_goal_node --ros-args --params-file ~/linorobot2_ws/src/nav_goal_node/config/locations.yaml`

# 6. Testen & Gebruik
### Een commando sturen

Je kunt de server simuleren door handmatig een bericht te publiceren.

`ros2 topic pub --once /move_command std_msgs/msg/String "data: 'kitchen'"`

### De status controleren

Kijk live mee wat de robot terug rapporteert.

`ros2 topic echo /fleet_status`

### Verwachte output:
YAML

     robot_id: "robot_1"
     status: 2  # (MOVING)
     description: "Heading to kitchen"
     target_node: "kitchen"
     ...

# 7. Troubleshooting
| Error Melding	| Waarschijnlijke Oorzaak | Oplossing |
| ------------- | ----------------------- | --------- |
| RTPS_TRANSPORT_SHM Error |	Geheugen vol door crashende nodes.	| Reboot de robot (sudo reboot). Dit is de enige betrouwbare fix. |
| Action server unavailable	| Nav2 (Terminal 2) is niet gestart of gecrasht.	| Controleer Terminal 2. Als die draait: zie punt hierboven (Reboot). |
| Message Filter dropping message	| Tijd loopt niet synchroon of CPU overbelast.	| Niet compileren (colcon build) terwijl de robot rijdt! Herstart alles. |