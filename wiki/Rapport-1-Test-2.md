# Test communicatie

Naam tester: 	Hele groep (Lars, Lars, Thomas, Jelle, Jarek, Ruben

Datum: 14/01/2026		

## Soort test(en)

Integratie test

## Beschrijving test

Deze test is een functionele test en is een must-have aangezien het basisfunctionaliteit bevat voor het functioneren van de robot.
Tijdens deze test worden de fleetmanager en 1 robot opgestart die door middel van zenoh bridges met elkaar verbonden zijn. De robot stuurt status updates naar de fleetmanager waar ze zichtbaar zullen zijn. Er wordt gecontroleerd of de berichten goed binnenkomen bij de fleetmanager en of de communicatie stabiel blijft gedurende deze periode.

## Component / Functie die wordt getest

* Zenoh bridges
* Status update node

## Test doel

Het doel van de test is controleren of de status updates van de robot bij de fleetmanager goed en consistent binnen komen door middel van zenoh.

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

In eerste instantie waren we vergeten om zenoh opnieuw op te starten wat betekende dat het niet werkte, maar na een snelle restart is de test vlekkeloos gelukt.

## Pass/Fail

* Er kan startinfo doorgestuurd worden naar de robot waarin een locatie en verdieping nummer worden ingevuld									X Pass    ☐ Fail
* De robot stuurt een status als de startinfo is ontvangen					X Pass    ☐ Fail
* De robot stuurt een status als de locatie onbereikbaar is					X Pass    ☐ Fail
* De robot stuurt een status als de locatie bereikt is						X Pass    ☐ Fail
* De status updates zijn zichtbaar op de fleetmanager					X Pass    ☐ Fail

Algemeen:
X Pass    ☐ Fail

## Test Environment

Lokale linux omgeving op de robot, fleetmanager en laptop (Ubuntu en Ros2 Jazzy)

## Opmerkingen / Notities / Tips

Notitie: NIET VERGETEN ZENOH OPNIEUW OPSTARTEN!!!
