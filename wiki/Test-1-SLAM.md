# Slam test

Naam tester: 	

Datum: 	

## Soort test(en)

Hardware test en Integratie test

## Beschrijving test

Deze test is een functionele test en is een must-have aangezien het basisfunctionaliteit bevat voor het functioneren van de robot.
Tijdens de test wordt de robot geplaatst in een onbekende omgeving. Er wordt dan handmatig met de controller een map gemaakt met de LIDAR door middel van SLAM. Nadat de robot de kaart heeft gemaakt wordt deze handmatig ingeladen, hierna moet de robot zichzelf door de omgeving kunnen navigeren met SLAM. Hierbij moet de robot obstakels ontwijken, hierbij gaat het om zowel bekende (De map) als onbekende obstakels (bijv. Een omgevallen doos). Onder de onbekende obstakels vallen ook bewegende obstakels, zoals mensen die rondlopen om te controleren of deze obstakels ook goed ontweken worden.

## Component / Functie die wordt getest

* Lidar

* Motoren

* Map maken

* SLAM

* Navigatie Functionaliteit (nav goal node)

* Ontwijken van obstakels

## Test doel

Het doel van deze test is om te verifiëren dat alle onderdelen die nodig zijn voor het autonoom rondrijden operatief zijn. De robot moet kunnen rijden en tegelijkertijd een betrouwbare kaart maken, zichzelf lokaliseren op basis van deze kaart en veilig te navigeren zonder obstakels te raken, deze kunnen bekend, onbekend of zelfs dynamisch zijn.

## Test Input(s)

Tijdens de test krijgt de robot constant data binnen van de lidar. Verder krijgt het ook nog een doelpose (goal pose) om naar te navigeren.

##  Stappenplan

* Maak een map door de robot te besturen met de logitech controller.

* Laad de map in op de robot

* Vul een goal pose in door middel van de nav goal node
 
## Verwachte uitkomst

De robot is in staat om een bruikbare kaart te maken van een onbekende ruimte. Op basis van deze kaart kan hij zich veilig door de ruimte navigeren en lokaliseren zonder obstakels te raken. Zowel bekende obstakels die op de kaart staan als onbekende obstakels moeten ontweken worden. Deze kunnen statisch of dynamisch zijn. Alle obstakels moeten tijdig gedetecteerd en veilig ontweken worden. 

Hier een lijst van verwachte dingen die correct uitgevoerd moeten worden om de volledige test als geslaagd te zien. Deze is in te vullen bij “pass/fail".
De robot kan handmatig rondrijden door middel van de logitech controller

* Er kan een map worden gemaakt

* De map kan worden opgeslagen

* De map kan worden ingeladen

* Er kan een locatie worden ingevuld op de nav goal node

* De robot rijd autonoom naar de goal pose

* De robot ontwijkt bekende obstakels (map)

* De robot ontwijkt statische onbekende obstakels (doos)

* De robot ontwijkt dynamische onbekende obstakels (rondlopende mensen)

## Echte uitkomst

Hier komt wat er echt is gebeurd tijdens de test

## Pass/Fail

* De robot kan handmatig rondrijden door middel van de logitech controller			☐ Pass    ☐ Fail

* Er kan een map worden gemaakt								☐ Pass    ☐ Fail

* De map kan worden opgeslagen								☐ Pass    ☐ Fail

* De map kan worden ingeladen								☐ Pass    ☐ Fail

* Er kan een locatie worden ingevuld op de nav goal node					☐ Pass    ☐ Fail

* De robot rijd autonoom naar de goal pose							☐ Pass    ☐ Fail

* De robot ontwijkt bekende obstakels (map)							☐ Pass    ☐ Fail

* De robot ontwijkt statische onbekende obstakels (doos)					☐ Pass    ☐ Fail

* De robot ontwijkt dynamische onbekende obstakels (rondlopende mensen)		☐ Pass    ☐ Fail

Algemeen:
☐ Pass    ☐ Fail

## Test Environment

Welke omgeving is gebruikt? (e.g. Lokaal, Docker container, Azure deployment)

## Opmerkingen / Notities / Tips

Eventuele observaties, vervolgacties of geconstateerde problemen






