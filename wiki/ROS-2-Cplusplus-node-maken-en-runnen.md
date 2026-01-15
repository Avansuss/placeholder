## 1. Ga naar je ROS 2 workspace
```
cd ~/linorobot2_ws/src
```

## 2. Maak een nieuw ROS 2 package aan
We maken een C++ node met ament_cmake en gebruiken rclcpp en std_msgs.
```
ros2 pkg create --build-type ament_cmake {NODE_NAAM}--dependencies rclcpp std_msgs
```

## 3. Controleer of de mappenstructuur klopt
Gebruik:
```
ls
```

Je zou nu dit moeten zien:
```
{NODE_NAAM}/
├── CMakeLists.txt
├── package.xml
└── src/
```

## 4. Ga naar de src map van je package
```
cd {NODE_NAAM}/src
```

## 5. Maak je C++ node bestand
```
sudo nano start_info_node.cpp
```

In deze file schrijf je je ROS 2 C++ code en sla je het op

## 6. Terug naar de workspace root

```
cd ~/linorobot2_ws
```

## 7.  Build alleen dit package
```
colcon build --packages-select start_info_node
```

Dit compileert alleen jouw node.

## 8. Source de workspace 
```
source install/setup.bash
```

Dit moet je elke keer doen wanneer:
- je code hebt aangepast
- je opnieuw hebt gebuild
- je een nieuwe terminal opent
Zie het als een “refresh” van ROS.

## 9. Run je node 
```
ros2 run start_info_node start_info_node
```

Als alles goed is:
- geen errors


## 10. Workflow samenvatting 
```
cd ~/linorobot2_ws
colcon build --packages-select start_info_node
source install/setup.bash
ros2 run start_info_node start_info_node
```
