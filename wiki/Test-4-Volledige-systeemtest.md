# Volledige systeem test
Naam tester: 	
<br>
Datum: 	

## Soort test(en)
Systeemtest
Intergratietest 

## Beschrijving test
Deze test is een functionele test en is een must-have aangezien het alle functionaliteiten om aan de eisen te voldoen uitvoert.
Alle onderdelen van het hele systeem worden hier getest. We maken een testomgeving die alle functionaliteiten van het systeem kan testen 



(afbeelding 1). Bij deze test maken we maps en laden we deze maps in op de robots. Hierna zetten we de robots op hun startposities. Voor ons is de startlocatie van de robot met goederenbak op locatie 1 aangezien dit de meeste mogelijkheid biedt voor testen met deze robot. Verder komt de lift robot op locatie 2 en de laatste robot op locatie 3. Deze locaties zijn te vinden in afbeelding 1.

<img width="2000" height="1000" alt="locaties" src="https://github.com/user-attachments/assets/176e4cd0-7f60-486c-b2b8-2aa2b043a7d1" />

### Afbeelding 1: Test opstelling met start locaties van de robots

Als alle robots op hun startlocatie staan en de maps zijn ingeladen kan de startinfo ingevuld worden. Hierna zou robot 1 naar robot 2 moeten rijden en zodra hij is aangekomen de goederen bak openmaken. Vervolgens klikt robot 2 (de lift robot) op de lift knop om de deur open te maken. Hierna gaat robot 2 de lift in en drukt hij op de knop die bij de juiste verdieping hoort. Daarna rijdt de lift robot naar robot 3. Die vervolgens weer doorrijdt naar de in de startinfo ingevulde eindlocatie.

## Component / Functie die wordt getest
Alle functies

## Test doel
Met deze test valideren we de gehele robotvloot en de werking hiervan, zodat we zeker weten dat aan alle eisen is voldaan en de vloot autonoom kan functioneren.

## Test Input(s)
De startinformatie
Arm besturing

## Stappenplan
Maak de testopstelling volgens de afbeelding 2

<img width="2000" height="1000" alt="base img" src="https://github.com/user-attachments/assets/6e418af8-a777-408f-bd34-882d93e4f4d8" />

### Afbeelding 2: testopstelling

Maak de maps van de locaties aangegeven in afbeelding 3. 

<img width="2000" height="1000" alt="Untitled" src="https://github.com/user-attachments/assets/ce822f05-0237-4266-b94d-af3df0b7ed51" />

### Afbeelding 3: testopstelling met map locaties

Laad de maps in op de bijbehorende robots (map 1 hoort bij robot 1 enz.)
Restart Zenoh
Vul de startinformatie in op de fleetmanager
Bestuur de arm handmatig naar de juiste liftknoppen
De rest gebeurd vanzelf

## Verwachte uitkomst
* De robot vloot functioneert autonoom nadat de startinformatie is doorgestuurd zonder errors of te stoppen zonder reden. 
* Bij “Open” gaat het bakje open en bij “Close” gaat het bakje dicht. Hierbij verwachten wij geen foutmeldingen en vertraging.
* De niet lift robots kunnen alle worden bestuurd door de logitech controller
* Alle maps kunnen correct worden gemaakt
* Alle maps kunnen correct worden opgeslagen
* Alle maps kunnen correct worden ingeladen op de juiste robots
* De startinfo kan worden ingevuld
* Alle robots ontvangen de startinfo
* De goederenklep van robot 1 functioneert correct
* Robots sturen correct status updates
* Robots ontvangen correct berichten
* Fleetmanager ontvangt status updates correct
* Fleetmanager verstuurt correct berichten
* De lift robot kan op knoppen drukken
* De camera detecteert de afstand van de correcte tracker
* De niet lift robots kunnen een goal pose doorkrijgen en daarnaartoe rijden
* De rijdende robots botsen tegen geen bekende, onbekende en dynamische obstakels

## Echte uitkomst
Hier komt wat er echt is gebeurd tijdens de test

## Pass/Fail
* De niet lift robots kunnen alle worden bestuurd door de logitech controller ☐ Pass    ☐ Fail

* Alle maps kunnen correct worden gemaakt ☐ Pass    ☐ Fail

* Alle maps kunnen correct worden opgeslagen ☐ Pass    ☐ Fail

* Alle maps kunnen correct worden ingeladen op de juiste robots ☐ Pass    ☐ Fail

* De startinfo kan worden ingevuld ☐ Pass    ☐ Fail

* Alle robots ontvangen de startinfo ☐ Pass    ☐ Fail

* De goederenklep van robot 1 functioneert correct ☐ Pass    ☐ Fail

* Robots sturen correct status updates ☐ Pass    ☐ Fail

* Robots ontvangen correct berichten ☐ Pass    ☐ Fail

* Fleetmanager ontvangt status updates correct ☐ Pass    ☐ Fail

* Fleetmanager verstuurt correct berichten ☐ Pass    ☐ Fail

* De lift robot kan op knoppen drukken ☐ Pass    ☐ Fail

* De camera detecteert de afstand van de correcte tracker	☐ Pass    ☐ Fail

* De niet lift robots kunnen een goal pose doorkrijgen en daarnaartoe rijden ☐ Pass    ☐ Fail

* De rijdende robots botsen tegen geen bekende, onbekende en dynamische obstakels	☐ Pass    ☐ Fail

Algemeen:
<br>		
☐ Pass    ☐ Fail

## Test Environment
Welke omgeving is gebruikt? (e.g. Lokaal, Docker container, Azure deployment)

## Opmerkingen / Notities / Tips
Eventuele observaties, vervolgacties of geconstateerde problemen
