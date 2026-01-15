IP-instellingen in [Ip ranges
](https://github.com/Avansuss/placeholder/wiki/Ip-ranges)

Nodes aanpassen en installeren
[ROS 2 Cplusplus node maken en runnen
](https://github.com/Avansuss/placeholder/wiki/ROS-2-Cplusplus-node-maken-en-runnen)

Robot13 maakt gebruik van een camera en een robotarm als extra aangesloten aan de seriële poorten van de Raspberry Pi. 
Robot12 maakt gebruikt van een ESP32 Feather v2 als extra aangesloten aan een seriële poort van de Raspberry Pi.

Het checken wat er aangesloten is aan de seriële poorten van de Raspberry Pi kun je checken door: 
```
ls /dev/ttyACM*
```

Tip: Haal kort de kabel van de poort die je wilt weten kort uit de Raspberry Pi. Run dan de code, doe de kabel terug in de Raspberry Pi en run de code alweer. Hierdoor zie je of de kabel nog werkt en welke seriële poort je gebruikt. 

Om de robots te starten, moeten alle nodes worden opgestart: 
- Nodes: [ROS Netwerkoverzicht
](https://github.com/Avansuss/placeholder/wiki/ROS-Netwerkoverzicht)
- de SLAM-nodes worden expliciet opgestart op deze manier: [SLAM Nodes Guide](https://github.com/Avansuss/placeholder/wiki/Robot-nodes-slam-etc-2.0)
- Micro-ROS-node: [Hardware‐integratie micro‐ROS servo motor
](https://github.com/Avansuss/placeholder/wiki/Hardware%E2%80%90integratie---micro%E2%80%90ROS-servo-motor)

Verdere informatie over de robotconfiguratie: 
- [ROS Netwerkoverzicht
](https://github.com/Avansuss/placeholder/wiki/ROS-Netwerkoverzicht)
- [Zenoh-bridges](https://github.com/Avansuss/placeholder/wiki/Opzetten-zenoh-bridge)
- [Communicatie/Interface beschrijving](https://github.com/Avansuss/placeholder/wiki/Communicatie-Interface-beschrijving)

Controleer ook de routerverbinding voor je de robots gebruikt, zodat je de robots kunt gebruiken tegelijkertijd met je WiFi: 
- [Router verbinding](https://github.com/Avansuss/placeholder/wiki/Connecting-the-router-mesh)


Andere specifieke details zijn terug te vinden in de rest van de pages. 