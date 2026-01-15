# Setup (Jazzy-methode)
Deze methode werkt **niet** op Raspberry Pi. <br>
Nodig voor verbinding: U2D2

In je omgeving voer je de volgende stappen uit:

Zorg ervoor dat je omgeving kan communiceren met de arm: <br>
* `sudo usermod -aG dialout $USER` <br>
(In plaats van USER moet je de daadwerkelijke gebruiker invullen, in ons geval rens) <br>
Hierna moet je de omgeving opnieuw opstarten

Maak een directory aan voor de bestanden:
* `mkdir -p ros2_ws/src` <br>
(Je kunt een andere naam gebruiken dan `ros2_ws`, mogelijk heb je al een directory met die naam, maar let wel op dat je dan ook in de komende commando's de naam vervangt)

Ga naar de directory:
* `cd ~/ros2_ws/src`

De volgende vier regels kun je als één geheel uitvoeren, dankzij de tekens `&& \`. <br>
* `git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \` <br>
`git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \` <br>
`git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \` <br>
`git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git`

Ga uit de `src`-directory: <br>
* `cd ~/ros2_ws`

* `sudo rosdep init` <br>
Waarschijnlijk krijg je hierna een foutmelding dat je het bestand `20-default.list` in `/etc/ros/rosdep/sources.list.d` moet verwijderen. doe dat met de volgende twee commando's: <br>
  * `cd /etc/ros/rosdep/sources.list.d` <br>
  * `sudo rm 20-default.list`

Vervolgens ga je terug naar `~/ros2_ws` en voer je `sudo rosdep init` opnieuw uit als het eerst niet gelukt was.

* `rosdep update` <br>
* `rosdep install --from-paths src -y --ignore-src` <br>

* `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

* `source ~/ros2_ws/install/setup.bash`

De drie regels hieronder kunnen weer in één keer uitgevoerd worden <br>
(Let op! In de tweede regel staat `ros2_ws`, vervang dit als je directory anders heet)
* `echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \` <br>
`echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && \` <br>
`echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc`

* `source ~/.bashrc`

* `ros2 run open_manipulator_bringup om_create_udev_rules`

Nu is je apparaat ingesteld om de robotarm te gaan gebruiken.

# Arm Gebruiken (Jazzy-methode)

Gebruik een USB-C kabel om je apparaat te verbinden met de U2D2 van de robotarm (het plaatje naast de arm).

Door dit in je omgeving te voeren zou de arm in een startpositie moeten gaan staan:
* `ros2 launch open_manipulator_bringup open_manipulator_x.launch.py`

Door de volgende regel in een **tweede terminal** in te voeren terwijl het vorige nog bezig is, kun je de arm met je toetsenbord besturen:
* `ros2 run open_manipulator_teleop open_manipulator_x_teleop`

Toetsen:
* `1`/`Q`: Arm Linksom/Rechtsom Draaien
* `2`/`W`: Eerste Scharnier ("Schouder") Buigen/Strekken
* `3`/`E`: Tweede Scharnier ("Elleboog") Buigen/Strekken
* `4`/`R`: Derde Scharnier ("Pols") Buigen/Strekken
* `O`/`P`: Grijper Open/Dicht <br>
(De grijper gaat open met de letter O, niet het cijfer nul)

# Teleop-bestand (Jazzy-methode)

Het is mogelijk extra functionaliteit toe te voegen aan teleop (het bestand waardoor je de arm met je toetsenbord kunt besturen). <br>
Je gaat naar het juiste bestand met het commando `nano ~/ros2_ws/src/open_manipulator/open_manipulator_teleop/open_manipulator_teleop/open_manipulator_x_teleop.py
` <br>
(Dit commando ziet er anders uit als je directory anders is ingericht)

Hier is een voorbeeld van een extra stukje code dat is toegevoegd (het stuk dat geselecteerd is): <br>
<img width="1359" height="1131" alt="Screenshot from 2026-01-07 14-55-04" src="https://github.com/user-attachments/assets/8b76ecb3-2baf-40a9-9f79-2a66d3dcfdb9" />

Deze code zorgt ervoor dat je de arm kunt strekken door op L te drukken. <br>
(Voor line indentation moet je spatie gebruiken. Tab gebruiken zorgt voor een error) 

Het valt je misschien op dat de toegevoegde code er anders uitziet dan de code die ervoor staat. Dit komt doordat we in onze code de arm een vaste positie geven om naartoe te bewegen (simpel gezegd, bijvoorbeeld "ga naar positie 0"). De andere code beweegt de arm omhoog of omlaag (meer iets als "ga naar {momentele positie} + 1").

Laten we meteen even doornemen wat een stukje bestaande code doet <br>
```
elif key == 'r':
   new_pos = max(
      self.arm_joint_positions[3] - self.max_delta, -1.5
   )
   self.arm_joint_positions[3] = new_pos
```
Wanneer R is ingedrukt, wordt er een vergelijking gedaan: <br>
{De momentele positie van scharnier 3} - {de stapgrootte van de beweging die in één tick uitgevoerd kan worden} <br>
(In andere woorden, de nieuwe positie waar de arm heen wilt gaan) <br>
Dit wordt voor de veiligheid vergeleken met: <br>
{De laagst mogelijke waarde die de positie van scharnier 3 mag hebben (-1.5) (anders draait de scharnier te ver)}

De scharnier gaat dan naar de grootste positie van de twee. Zo draait de scharnier niet te ver.

# Setup (Arduino-methode)
Deze methode werkt **wel** op Raspberry Pi. <br>
Nodig voor verbinding: OpenCR

Deze eerste paar stappen kunnen gewoon met je laptop, je hoeft alleen wat code op de OpenCR te krijgen.

Open je Arduino IDE en voeg in de Board Magager "OpenCR" toe, en bij File>Preferences>Additional boards manager URLs voeg je `https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json` toe.

Sluit de OpenCR aan op je computer, en selecteer in de IDE het juiste bord en de juiste port.

Open het voorbeeldbestand File>Examples>OpenManipulator>Chain>open_manipulator_chain en zet het op je OpenCR. Mogelijk uploadt het bestand alleen in "safe mode". Houd op je OpenCR-bord "PUSH Sw2" ingedrukt, houd "Reset" ingedrukt, laat "Reset" los, en laat even daarna pas "PUSH Sw2" los. Als het goed is gaat het lampje met "STATUS" erbij snel knipperen. Probeer hierna nog een keer te uploaden. Bij een correcte upload komt er "[OK] Download" in de terminal in beeld.

Ssh naar je robot, en download "Processing" (een soort IDE) dmv het commando `sudo snap install processing --classic`. Hier heb je snap voor nodig. Als je snap nog niet hebt, doe je eerst even `sudo apt install snapd`.

Je moet nog een bestand downloaden op je robot dmv `git clone https://github.com/ROBOTIS-GIT/open_manipulator_processing.git`.

Commando `export DISPLAY=:0`, zo kun je straks op je scherm vensters zien die runnen op je robot.

Met `nano /etc/ssh/sshd_config` kun je kijken of er "X11Forwarding yes" in sshd_config staat, dit is ook nodig voor de vensters.

Ssh opnieuw naar je robot, maar voeg nu `-X` toe achter het ssh-commando. Dat is het laatste dat nodig is voor de vensters.

Open Processing met `snap run processing`

Als het goed is opent na een tijdje Processing op je laptop. De vensters werken wel erg traag

Ga naar Tools>Add Tool, ga naar Libraries en installeer "ControlP5"

# Arm Gebruiken (Arduino-methode)

Zorg ervoor dat je op je robot zit, en `-X` achter je commando hebt gezet toen je ssh ernaar uitvoerde.

Open in Processing het bestand dat je eerder gedownload hebt, de directory is open_manipulator_processing>Chain>Chain.pde

Zorg ervoor dat je OpenCR-bord stroom krijgt via de "SMPS In" (ronde port in een van de hoeken van het bord), en sluit hem op je Raspberry Pi aan met een USB-A>Micro-USB-kabel.

Let op: als de Micro-USB zit aangesloten en "SMPS In" niet (of de schakelaar staat uit), probeert het bord stroom via de Micro-USB-kabel uit de Raspberry Pi te halen. Dit kan ervoor zorgen dat andere delen van je robot niet genoeg stroom krijgen, en mogelijk is het zelf slecht voor de OpenCR.

Wanneer je Chain.pde runt, krijg je een GUI waarmee je de arm kunt aansturen.

Als het niet werkt, moet je misschien de juiste USB-port kiezen. In Chain.pde heb je "void setup" met daarin "connectOpenCR(0)" door de 0 een hoger getal te maken, pas je aan welke port er gebruikt wordt.

Deze methode werkt nog steeds niet consequent. Wanneer je Processing op je laptop installeert en runt, met de Micro-USB van de OpenCR verbonden met je laptop ipv de Raspberry Pi, werkt het veel sneller en beter. Dit komt doordat X11 (waarmee de vensters van de Raspberry Pi op je laptop verschijnen) erg zwaar is voor een Raspberry Pi. We wilden echter een methode vinden die werkt vanuit de Raspberry Pi, en dat doet deze methode wel.

# Verder met de Arduino-methode

We wilden graag als volgende stap de OpenCR opdrachten kunnen geven met een C++ bestand, in plaats van met een volledig voorgeprogrammeerde GUI, zodat we de arm in het ROS2 node-netwerk konden verwerken. Het is gelukt zelf een Arduino-bestand voor de OpenCR te schrijven waardoor de scharnieren op slot gingen (meestal wanneer je begint met het aansturen van de arm, gaan alle scharnieren op slot, zodat je de arm niet meer handmatig kunt bewegen, dus dat was een goed teken), maar de arm verder bewegen leek niet te werken. Het was verrassend lastig om informatie over de OpenManipulator-X te vinden naast de voorgeprogrammeerde software, die vaak simpel te gebruiken is, maar lastig te ontleden. Ook was het niet duidelijk of de OpenCR berichten kon ontvangen vanuit C++, dus uiteindelijk hebben wij het werken aan een arm-node opgegeven.