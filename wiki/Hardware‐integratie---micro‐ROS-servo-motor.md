# Servo motor aansturen met micro-ROS op de robot

Om de servo motor te laten werken met micro-ROS op de robot zijn de volgende stappen nodig:

## 1. Ontwerp en code
- Maak eerst een ontwerp om de servo te laten werken met code.  
  - Wij hebben hiervoor een knop gebruikt die de servo aanstuurt.  
  - Later wordt deze knop vervangen door een publisher die de data publiceert waarmee de servo een bepaalde hoek draait.

<img width="600" height="450" alt="image" src="https://github.com/user-attachments/assets/544e906b-cbb1-4ae2-a656-ee9af9789a2b" />

## 2. ESP32 lokaal voorbereiden
- Sluit de ESP32 aan op je laptop.  
- Schrijf de code in Arduino IDE.  
  - **Belangrijk:** gebruik de micro-ROS library in je code!  
- Flash de code naar de ESP32 Feather v2.

## 3. ESP32 aansluiten op de robot
- Zodra de code geflasht is, sluit je de ESP32 aan op de Raspberry Pi van de robot.  

## 4. Controleer de seriële poort
- Open een terminal (Terminal 1) op de Raspberry Pi en controleer welke poort de ESP gebruikt:

```
ls /dev/ttyACM*
```

- Koppel de kabel kort los en run het commando opnieuw om te zien welke poort verdwijnt en weer verschijnt.
- Onthoud deze poort; deze heb je nodig voor de micro-ROS agent.
## 5. Start de micro-ROS agent
- Run de micro-ROS agent met het juiste poortnummer:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM{poortnummer} -v6
```

- Vervang {poortnummer} door het poortnummer dat je in stap 4 hebt gevonden.
- -v6 zorgt voor uitgebreide logging.
Probleem oplossen
- Als je alleen dit ziet:

```
[info] TermiosAgentLinux.cpp | init | running...
[info] Root.cpp | set_verbose_level | logger setup
```

- Dan is er nog geen client gestart. Mogelijke reden: Arduino IDE of een andere seriële monitor (bijv. sudo screen /dev/ttyACM{poortnummer} 115200) staat nog open. Sluit deze eerst af voordat je de agent start.
## 6. Controleer of de client werkt
- Zodra alles correct werkt, zie je in de agent logs iets als:

```
[info] create_client            | create
[info] establish_session        | session established
[debug] send_message            | [** <<SER>> **]
[debug] recv_message            | [==>> SER <<==]
```

- Dit betekent dat de ESP32 micro-ROS client succesvol verbinding heeft gemaakt.
## 7. Controleer de node
- Kijk of de node is aangemaakt en draait:

```
ros2 node list
```

- Je zou de node moeten zien die je in de code hebt aangemaakt.
## 8. Publish naar de subscriber
- Je kunt nu data sturen naar de ESP32 zodat de servo draait:

```
ros2 topic pub /servo_target std_msgs/msg/Int32 "{data: 180}"
```

- data is het aantal graden waarin de servo moet draaien.
- Als het goed is, beweegt de servo mee met de gepubliceerde waarde.

## 9. Keuze van microcontroller (ontwerpoverweging)

Voor de aansturing van de servo is gekozen voor een ESP32 Feather v2 in combinatie met micro-ROS. Deze keuze is gemaakt op basis van de volgende overwegingen:

- De ESP32 ondersteunt FreeRTOS native, wat noodzakelijk is voor niet-blokkerende ROS-communicatie.

- De ESP32 beschikt over hardwarematige PWM-uitgangen, wat zorgt voor stabiele en nauwkeurige servo-aansturing.

- Door hardware-actuatie te scheiden van de Raspberry Pi blijft de ROS 2 navigatie-stack ontlast en robuuster.

- Eén microcontroller is voldoende omdat slechts één actuator lokaal real-time aangestuurd hoeft te worden.

Deze architectuur maakt het systeem schaalbaar: extra actuatoren kunnen later worden toegevoegd via aanvullende micro-ROS nodes.


## 10. Keuze van actuator en input (relevantie voor de casus)

Als actuator is gekozen voor een servo motor, omdat deze:

- een vaste hoekpositie kan aannemen zonder complexe terugkoppeling

- eenvoudig aan te sturen is via PWM

- geschikt is voor mechanische taken zoals het openen/sluiten van een klepmechanisme. 


Tijdens ontwikkeling is een drukknop toegevoegd als lokale input:

- voor testdoeleinden

- als uiteindelijke noodknop die de klep opent/sluit zonder gebruik van een publisher.


In de uiteindelijke toepassing wordt de servo aangestuurd via ROS-berichten vanuit een publisher in het robotnetwerk.


## 11. Toepassing van FreeRTOS-concepten

De ESP32 maakt gebruik van FreeRTOS voor het uitvoeren van meerdere taken binnen één microcontroller. Binnen de implementatie:

- draait de micro-ROS executor in een aparte FreeRTOS task,

- blijft de hoofdloop beschikbaar voor servo-aansturing en inputverwerking,

- wordt blokkerend gedrag voorkomen.

Deze scheiding verhoogt de stabiliteit en responsiviteit van zowel ROS-communicatie als hardware-aansturing.


## 12. Gebruik van bestaande bibliotheken

Voor de implementatie is gebruikgemaakt van bestaande en bewezen bibliotheken:

- micro_ros_arduino voor ROS 2 communicatie op embedded hardware,

- rclc voor het opzetten van nodes, executors en subscriptions,

- ESP32Servo voor betrouwbare PWM-aansturing van de servo.

Door gebruik te maken van deze bibliotheken wordt hardware-afhankelijk gedrag correct afgehandeld en blijft de code onderhoudbaar en uitbreidbaar.


## 13. Relatie met het ROS-netwerk

De micro-ROS servo node functioneert als ROS 2 subscriber binnen het bestaande netwerk.

- De node ontvangt ROS-berichten via een micro-ROS agent op de Raspberry Pi.

- De communicatie verloopt via standaard ROS 2 topics.

- De node voegt fysieke hardware-actuatie toe zonder bestaande ROS-nodes te wijzigen.

Hiermee sluit de microcontroller naadloos aan op de ROS-architectuur van de robot.