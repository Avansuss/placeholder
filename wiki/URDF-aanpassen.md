# urdf aanpassen

**Gemeten lengtes van de robot in meter (iedere waarde in een urdf is in meter):**

Base length = 0,36 m

Base width = 0,305 m

Base height = 0,082 m

Wheel radius = 0,07 m

Wheel width = 0,03 m

## inloggen op de robot

Om de urdf aan te passen moet je op de robot ingelogd zijn.

Dit door met het juiste wifi netwerk te verbinden en vervolgens in de command line:

```ssh rens```

Als dit niet werkt probeer dan:

```ssh rens@{Ip-adres}```

Het ip-adres behorend bij het wifi netwerk kan je hier vinden: [Ip ranges](https://github.com/Avansuss/placeholder/wiki/Ip-ranges)

Vul hierna het wachtwoord in om in te loggen.

## Openen van urdf

Navigeer naar de locatie waar de urdf is opgeslagen met het volgende commando:

```cd ~/linorobot2_ws/install/linorobot2_description/share/linorobot2_description/urdf```

Open vervolgens het bestand ```2wd_properties.urdf.xacro``` met het volgende commando:

```nano 2wd_properties.urdf.xacro```

## Opslaan van de file

Als je alles hebt aangepast klik dan op ^x (ctrl + x) om de file te sluiten. Klik hierna op y om op te slaan. En vervolgens op enter om de naam op te slaan (naam niet veranderen). Je kan ook gewoon ctrl + s doen en vervolgens het bestand weer sluiten met ctrl + x.

Dit zijn alle stappen om de urdf aan te passen.



