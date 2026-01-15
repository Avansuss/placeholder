# Test goederenbak

Naam tester: 	

Datum: 	

## Soort test(en)

Integratie test

## Beschrijving test

Deze test is een functionele test en is een must-have aangezien het basisfunctionaliteit bevat voor het functioneren van de robot.
Goederenbak openen en sluiten via micro-ros servo-aansturing zodat de goederen veilig vervoerd kunnen worden.

## Component / Functie die wordt getest

* Goederenbak node
* Micro-Ros code op de microcontroller
* Servo- aansturing voor goederenbak

## Test doel

Controleer of de goederenbak correct opent en sluit wanneer een ROS 2 commando via micro-ROS wordt ontvangen.

## Test Input(s)
* Ros2 bericht verstuurd via een bepaald topic
* Message type: std_msgs/String
Inputwaarden:
* “Open” → servo draaien naar de open positie
* “Close” → servo’s draaien naar de gesloten positie

## Stappenplan

* Start de robot op
* Geef het commando om de klep open te maken
* Neem waar wat er gebeurd
* Geef het commando om de klep te sluiten
* Neem waar wat er gebeurd
* Druk op de noodknop
* Neem waar wat er gebeurd

## Verwachte uitkomst

Bij “Open” gaat het bakje open en bij “Close” gaat het bakje dicht. Hierbij verwachten wij geen foutmeldingen en vertraging.

Hier een lijst van verwachte dingen die correct uitgevoerd moeten worden om de volledige test als geslaagd te zien. Deze is in te vullen bij “pass/fail".

* Goederenklep gaat open wanneer het open commando wordt gegeven
* Goederenklep sluit wanneer het sluit command wordt gegeven
* Goederenklep gaat open als de noodknop is ingedrukt

## Echte uitkomst

Hier komt wat er echt is gebeurd tijdens de test

## Pass/Fail

* Goederenklep gaat open wanneer het open commando wordt gegeven			☐ Pass    ☐ Fail
* Goederenklep sluit wanneer het sluit commando wordt gegeven				☐ Pass    ☐ Fail
* Goederenklep gaat open als de noodknop is ingedrukt					☐ Pass    ☐ Fail

Algemeen:
☐ Pass    ☐ Fail

## Test Environment

Welke omgeving is gebruikt? (e.g. Lokaal, Docker container, Azure deployment)

## Opmerkingen / Notities / Tips

Eventuele observaties, vervolgacties of geconstateerde problemen
