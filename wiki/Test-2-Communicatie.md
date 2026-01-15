# Test communicatie

Naam tester: 	

Datum: 	

## Soort test(en)

Integratie test

## Beschrijving test

Deze test is een functionele test en is een must-have aangezien het basisfunctionaliteit bevat voor het functioneren van de robot.
Tijdens deze test worden de fleetmanager en 1 robot opgestart die door middel van zenoh bridges met elkaar verbonden zijn. De robot stuurt status updates naar de fleetmanager waar ze zichtbaar zullen zijn. Er wordt gecontroleerd of de berichten goed binnenkomen bij de fleetmanager en of de communicatie stabiel blijft gedurende deze periode.

## Component / Functie die wordt getest

* Zenoh bridges
* Status update node
* Test doel
* Het doel van de test is controleren of de status updates van de robot bij de fleetmanager goed en consistent binnen komen door middel van zenoh.

## Test Input(s)

Tijdens deze test stuurt de robot status-updates naar de fleetmanager. Deze status update bevat simpele informatie, zoals idle of busy. Dit bericht wordt gepubliceerd op een vooraf bepaald ros2 topic waarop de fleetmanager luistert om het bericht te ontvangen.

## Stappenplan

* Start de fleetmanager en de robot en zorg dat zenoh goed opstart
* Vul de startinformatie in op de fleetmanager
* Neem het resultaat waar

## Verwachte uitkomst

De verwachte uitkomst is dat de fleetmanager de status updates van de robot binnenkrijgt zonder fouten of merkbare vertraging. De ontvangen statuswaarde komt overeen met de verzonden waarde en wordt correct weergegeven in de logs of via een ROS 2 topic-echo. Er treden geen verbindingsproblemen of foutmeldingen op.

Hier een lijst van verwachte dingen die correct uitgevoerd moeten worden om de volledige test als geslaagd te zien. Deze is in te vullen bij “pass/fail".

* Er kan startinfo doorgestuurd worden naar de robot waarin een locatie en verdieping nummer worden ingevuld
* De robot stuurt een status als de startinfo is ontvangen
* De robot stuurt een status als de locatie onbereikbaar is
* De robot stuurt een status als de locatie bereikt is
* De status updates zijn zichtbaar op de fleetmanager

## Echte uitkomst

Hier komt wat er echt is gebeurd tijdens de test

## Pass/Fail

* Er kan startinfo doorgestuurd worden naar de robot waarin een locatie en verdieping nummer worden ingevuld									☐ Pass    ☐ Fail
* De robot stuurt een status als de startinfo is ontvangen					☐ Pass    ☐ Fail
* De robot stuurt een status als de locatie onbereikbaar is					☐ Pass    ☐ Fail
* De robot stuurt een status als de locatie bereikt is						☐ Pass    ☐ Fail
* De status updates zijn zichtbaar op de fleetmanager					☐ Pass    ☐ Fail

Algemeen:
☐ Pass    ☐ Fail

## Test Environment

Welke omgeving is gebruikt? (e.g. Lokaal, Docker container, Azure deployment)

## Opmerkingen / Notities / Tips

Eventuele observaties, vervolgacties of geconstateerde problemen
