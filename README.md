# Useless-Kitty-Machine
Converting a Kitty Money Box into a Useless Machine


Useless Kitty Box

 







Markus Kraft – Maximilian Kätscher - FSMT2 – 24.03.2021
 
Inhaltsverzeichnis

Inhaltsverzeichnis	2
1	Einleitung	3
2	Komponenten	4
2.1	Arduino	4
2.1.1	Technische Daten	5
2.2	Servomotoren	6
2.2.1	Technische Daten	6
2.3	3D Druckteile	7
2.3.1	Entwicklung des Inlays	8
2.4	Weitere Komponenten	9
3	Schaltplan	10
4	Zusammenbau	11
5	Programmierung	16
5.1	Programmieroberfläche	16
5.2	Der Programmcode	17
5.2.1	Initialwerte	17
5.2.2	Kalibrierung und Einstellung der Hebel.	18
5.2.3	Setup	19
5.2.4	Das eigentliche Programm	19
5.2.5	Übertragen des Programmcodes auf den Mikrocontroller	21
5.3	Anschluss-Pins	21
5.4	Umsetzung auf dem Raspberry Pi	22
6	Bedienung	24
7	Kostenaufstellung	25
8	Abbildungsverzeichnis	26
9	Literaturverzeichnis	27
10	Anhänge	28


 
1	Einleitung

Eine „Useless Box“ bzw. „Useless Machine“ kann normal nicht viel und hat auch keinen wirklichen Nutzen, außer vielleicht den Benutzer zu erfreuen. Denn alles was sie macht, ist einen Hebel umzulegen, den zuvor der Benutzer umgelegt hat. Das war’s. Das hält Bastler allerdings nicht davon ab unzählige Anleitungen, Videos und Varianten der Useless Machine zu entwickeln. Egal ob aus Holz, Kunststoff oder 3D-gedruckt. Mit einem Schalter, mit mehreren Schaltern, mit LEDs, mit Sound oder diversen anderen Features. Im Internet und auf YouTube findet man viele Inspirationen.

Für dieses Projekt wurde eine elektrische Spardose verwendet, bei der normalerweise eine Katze eine Münze mit ihrer Tatze von einer Münzablage in den Münzbehälter zieht. Die Demontage der Spardose zeigte, dass diese Bewegung durch einen kleinen Elektromotor und in komplexe Anordnung von Hebel bewerkstelligt wurde. Für den Umbau zu einer Useless Machine sollte dahingegen die Konstruktion relativ einfach werden und dafür die Steuerung komplexer. So sollte es mehrere Verhaltensweisen der Useless Machine geben, die zyklisch oder zufällig nach Umlegen des Schalters ausgeführt werden sollten.

Da der verfügbare Raum durch die Spardose begrenzt war, musste auf kleinem Raum konstruiert werden. Obwohl die Steuerung komplex sein sollte, konnte sie durch einfachen Programmcode umgesetzt werden, weswegen sich für ein kleines Arduino Nano Mikrocontroller Board als Steuereinheit entschieden wurde.

Für die Bewegung des Deckels und des Arms wurden kleine Servomotoren verwendet, die üblicherweise im Modellbau eingesetzt werden.

Im Folgenden wird beschrieben, wie die Spardose modifiziert wurde und welche Komponenten und Schritte für den Nachbau notwendig sind. 
2	Komponenten

2.1	Arduino

Arduino ist eine Open Source Elektronikplattform. Arduino Boards sind in der Lage Eingangssignale zu verarbeiten und Ausgangssignale zu schalten. Man kann dem Board Anweisungen geben, indem man einen Satz von Befehlen auf den Mikrocontroller des Boards überträgt. Hierfür verwendet man die Arduino Programmiersprache und die Arduino Software (IDE). Der Mikrocontroller basiert auf dem ATmega328-Controller. Der Arduino wird mit Gleichspannung betrieben, entweder über einen der Versorgungs-Pins (Pin 30: 6-20V, ungeregelt / Pin 27: 5V geregelt) oder über den integrierten MINI-USB-B-Anschluss. Als Spannungsquelle wird automatisch die Quelle mit der höchsten Spannung ausgewählt.

Der verbaute ATmega 328 verfügt über einen Speicher von 32 KB, 2 KB SRAM und 1 KB EEPROM.

Der Arduino Nano verfügt über diverse Pins wie beispielsweise digitale und analoge Pins. Einzelne Pins haben dabei spezielle Funktionen, auf die hier nicht im Detail eingegangen wird. Weitere Informationen hierzu findet man auf der Website von Arduino: https://www.arduino.cc.

Für das Projekt wurden die digitalen Pins verwendet. Jeder der 14 digitalen Pins des Arduino Nano kann sowohl als Input oder Output genutzt werden. Die Pins arbeiten dabei mit einer Spannung von 5 Volt. Jeder Pin kann maximal 40 mA empfangen und hat einen internen Pull-Up-Widerstand von 20-50 kOhm verbaut, der standardmäßig ausgeschaltet ist.

 
2.1.1	Technische Daten

Mikrocontroller 	ATmega328
Betriebsspannung	5 V
Stromverbrauch	19 mA
Flash Speicher 	32 KB von denen 2 KB vom Bootloader verwendet werden.
SRAM 	2 KB
Taktfrequenz	16 MHz
Analoge IN Pins 	8
DC Strom pro I/O Pin 	40 mA (I/O Pins)
Eingangsspannung	7-12 V
Digitale I/O Pins 	22 (6 davon für PWM)
Leiterplatinengröße	18 x 45 mm
Gewicht 	7 g

 	HINWEIS

Für dieses Projekt wurde kein original Arduino Controller verwendet, sondern ein Funduino NANO R3 Mikrocontroller, welcher über die gleiche Architektur verfügt und vollständig kompatibel ist.


 
2.2	Servomotoren

 
Abbildung 3  Servo SG90 TowerPro [3]

2.2.1	Technische Daten

Modulation	Analog
Betriebsspannung	4.0 bis 7.2 Volts
Drehmoment	4.8V: 1.2 kg/cm
Geschwindigkeit	4.8V: 0.12 sec/60°
Drehbereich	180°
Pulszyklus	ca. 20 ms
Pulsweite	500-2400 μs
Gewicht	9.0 g
Maße:	
Länge:	22,0 mm
Breite: 	11,5 mm
Höhe:	27,0 mm
Länge Verbindungskabel	248,0 mm
Motor Typ	3-polig
Getriebe Typ	Plastik
Anschluss	Universal "S" Type

Kabelfarben: Rot = Batterie (+), Braun = Batterie (-), Orange = Signal
 
2.3	3D Druckteile

Um die gesamte Elektronik und vor allem die Servomotoren in der Box richtig zu verstauen wurde ein komplettes Inlay konstruiert und anschließend mit einem 3D-Drucker ausgedruckt.
Zusätzlich wurden für die Servomotoren Hebelarme entworfen. Ein Arm, um die Klappe der Box zu öffnen und ein Weiterer, um die Tatze der Katze zu imitieren.
Damit der Servomotor den Schalter zuverlässig umlegen kann wurde eine Verlängerung konstruiert und ebenfalls, wie die restlichen Teile, mit Hilfe des 3D-Druckers ausgedruckt.

 	HINWEIS

Das 3D-Modell wurde an die verwendeten Bauteile angepasst. Sollten andere Bauteile verwendet werden, muss das 3D-Modell entsprechend angepasst werden.

 

2.3.1	Entwicklung des Inlays

Inlay V1:
Die Außenmaße der Bodenplatte, sowie Maße der Verschraubpunkte wurden abgenommen und aus diesen Maßen ein Inlay konstruiert.

Da die Maße nicht genau entnommen werden konnten und das Bauteil weder symmetrisch ist noch gerade Maße besitzt, passte der erste Entwurf nicht und musste entsprechend überarbeitet werden.


	 
Inlay V2:
Nachdem die Maße für die Stützen korrigiert wurden und anschließend passten, wurden zusätzlich auf das Inlay Halterungen für die Servomotoren konstruiert.
Für diese wurde der Servomotor im 3D-Modell nachmodelliert und die Baugruppe an diesen angepasst. 

In dieser Version passten nun die Stützen für die Verschraubung und die Halterungen der Servomotoren. Um die Useless-Machine testen zu können wurde in die Grundplatte eine Aussparung gefräst, um die Kabel herauszuführen, da zu diesem Zeitpunkt die Elektronik noch nicht in der Box verstaubar war.
	 
Inlay V3:
Mit der dritten Version des Inlays wurden alle Schwachstellen der vorherigen Inlays beseitigt und zusätzliche Anpassungen an die final verbauten Komponenten eingefügt, sodass die Kabel und das Board hinter einer Wand verstaut werden konnten. Außerdem wurde die Batteriebox im Boden versenkt, um Platz zu sparen und damit der Ein/Aus-Schalter von der Unterseite aus erreichbar ist.
Zudem kann durch die Öffnung im Boden der Arduino USB-Anschluss zur Programmierung erreicht werden.
Des Weiteren sind nun keine Kabel mehr im Innenraum, welche sich mit den beweglichen Hebelarmen verheddern könnten.
	 

 
2.4	Weitere Komponenten

1	Kondensator	220 µF, 10 V
1	Breadboard 	170 Steckplätze, 45 x 34,5 x 8,5mm
1	Kippschalter 	Kippschalter, Ein-Aus, DPDT
1	Batteriehalter 	4x AA Mignon, Ein-Aus-Schalter, lose Kabelenden
4	AA Akkus	1.2V, 2800 mAh

 

3	Schaltplan

 


 
 
4	Zusammenbau

1.	Zerlegen der Box.
Nach Entfernen der Schrauben im Boden der Box das alte Inlay entfernen.
Nun sollte die Box folgendermaßen aussehen:
 

2.	Schalter aus dem Deckel drücken. Dabei auf die Haltenasen aufpassen.
 

3.	Mit einem Schleifer die Führung des alten Schalters kürzen und abschleifen. Mit einem Bohrer die vorhandene Bohrung weiten, sodass der Schalter montierbar ist. 
 			 
vorher							nachher

 
4.	In den weißen Druckknopf muss entsprechend der Schaltergröße ein Loch gebohrt und auf der Rückseite mit einem Schleifer die Führung entfernt werden.
 			 
vorher							nachher

5.	Jumperkabel an den Schalter löten und wie im Foto dargestellt verbauen. Weißen Druckknopf wieder einsetzen.
 
   

 

6.	Gedruckten Schalter-Aufsatz montieren.
Um die Verlängerung zu montieren muss zuerst die Gummispitze des Schalters mit einer Zange abgezogen werden. Anschließend muss der selbstgedruckte Schalter aufgeschoben werden. 

7.	Batteriebox-Kabel durch die dafür vorgesehene Öffnung in den hinteren Bereich des Inlays verlegen und anschließend die Box mit doppelseitigem Klebeband auf dem Inlay befestigen. 
Achtung: Es ist darauf zu achten, dass der Ein/Aus-Schalter durch die dazugehörige Öffnung im Boden schaut. 
 

8.	Hebelarme auf die Servomotoren verschrauben. (Gerade für die Klappe, Gebogen für die Tatze). 
Achtung: Vor dem Anschrauben der Hebelarme an die Servomotoren Einbaulage prüfen und Hebelarme entsprechend ausrichten.

9.	Servomotoren auf dem gedruckten Inlay montieren. Einbaulage siehe Foto. 

10.	Servomotor-Kabel der Tatze wie auf dem Foto ersichtlich verlegen und durch die dafür vorgesehene Öffnung in den hinteren Bereich des Inlays führen. 

11.	Nun das Breadboard entsprechend dem Schaltplan mit den Jumperkabeln verdrahten. 
Achtung: der Kondensator muss richtig herum eingebaut werden! Die Dioden müssen wegen des beschränkten Platzes entsprechend gekürzt werden.
  			 

12.	Nun das Breadbord mittels des doppelseitigen Klebebands befestigen.
Anschlusskabel der Batteriebox-Kabel hinter dem Arduino und unter dem Breadboard verlegen und lt. Schaltplan anschließen.
 


13.	Die Servomotoren müssen nun entsprechend des Schaltplans angeschlossen und die Kabel sauber verlegt werden.
 


14.	Schalter anschließen und lt. Schaltplan mit dem Arduino verbinden.

15.	4 AA-Batterien einlegen und das Inlay in die Box schieben. 
Achtung: Es ist darauf zu achten, dass die Kabel nicht gequetscht werden!
 


16.	Zuletzt das neue Inlay mit den Schrauben des alten Inlays mit dem Deckel verschrauben.

 
5	Programmierung

5.1	Programmieroberfläche

Der Arduino Nano wird über die Arduino Software programmiert, welche kostenlos auf der Website von Arduino zur Verfügung gestellt wird.

 
Neben dem Prüfen und Kompilieren des Programmcodes (P), wird die Software verwendet, um den Programmcode auf den Mikrocontroller zu schreiben (è).

 

Bevor man den Programmcode auf den Arduino Nano schreiben kann, müssen im Menüpunkt „Werkzeuge“ noch oben angegebene Werte konfiguriert werden. Der Screenshot oben zeigt die Konfiguration auf einem MacOS-Betriebssystem mit einem Arduino Nano, welches über das mitgelieferte USB-Kabel an den Rechner angeschlossen ist. Für die Konfiguration auf einem Windows- oder Linux-Betriebssystem findet man im Internet ausführliche Anleitungen und Problemlösungen.

Da auf dem verwendeten Board ein CH340 Chip verbaut ist, musste als Prozessor „ATmega328P (Old Bootloader)“ verwendet werden. Bei anderen Boards kann diese Einstellung abweichen.

5.2	Der Programmcode

DISCLAIMER

Für die einzelnen Methoden des Programms wurde der Code des Nutzers „viorelracoviteanu“ von der Arduino-Projekt-Website verwendet und angepasst.

Die Dokumentation ist zu finden unter: 
https://create.arduino.cc/projecthub/viorelracoviteanu/useless-box-with-arduino-d67b47

5.2.1	Initialwerte

Das Arduino macht die Ansteuerung von Servomotoren einfach, da es eine eigene Bibliothek hierfür zur Verfügung stellt. Diese wird als erstes über folgenden Befehl in das Programm importiert.

#include <Servo.h>


Danach werden zwei Servomotoren deklariert. Einer zur Ansteuerung des Servomotors der Hand, einer zur Ansteuerung des Servomotors des Deckels.

Servo handServo;
Servo boxServo;


Als nächstes werden die Eingangs- und Ausgangs-Pins deklariert. In unserem Beispiel wurde der Eingang des Kippschalters auf Pin 2 gelegt und die Ausgänge der Servomotoren auf die Pins 7 und 9.

int switchPin = 2;
int handServoPin = 7;
int boxServoPin = 9;


Um später die einzelnen Methoden, die beim Umlegen des Kippschalters abgespielt werden sollen, auszuwählen wird eine Variable action deklariert, die später entweder zufällig ausgewählt oder durch einfaches Hochzählen durchlaufen wird.

action = random(1, 16);	action++;

Der Zustand des Schalters wird in der Variable switchStatus gespeichert.

int switchStatus = 0;
int action = 1;


Um die Ansteuerung zu erleichtern, werden spezifische Winkel definiert, anstatt Winkelwerte in den Befehl der Servoansteuerung zu schreiben. So muss man bei Kalibrierung der Winkelwerte diese nur einmal im Kopf des Programmcodes ändern, statt bei jeder Eingabe der Winkel. Bei der Kalibrierung der für diese Dokumentation zugrundeliegenden Box ergaben sich folgende Werte.

int switchOff = 30;
int switchDown = 140;
int switchPos1 = 130;
int switchPos2 = 80;
int switchPos3 = 60;

Das gleiche kann man mit dem Winkel des Deckels machen. Hier gibt es keine Zwischenstellungen, nur „auf“ und „zu“.
int zu = 50;
int auf = 100;


Beim Nachbau der Box können so die Winkel schnell und einfach kalibriert werden.

5.2.2	Kalibrierung und Einstellung der Hebel.

Die Hebelarme sind so zu montieren, dass die o.g. Positionen von diesen auch erreicht werden. Dabei ist darauf zu achten, dass die Hebel nicht über die Endpositionen hinausfahren. Hierfür sollte man die Ansteuerung der verbauten Servomotoren testen, bevor die Box auf das Inlay montiert wird.
Der Hebel für den Deckel muss manuell auf Position „zu“ gefahren werden. Der Hebel soll so montiert sein das er bei geschlossenem Deckel an dem Katzenkopf anliegt, den Deckel der Box aber noch nicht anhebt.
Der Hebelarm für den Schalter kann zuerst grob justiert werden. 
Für eine Feinjustierung der Montageposition und der anzusteuernden Winkel empfiehlt sich ein einfaches Programm, welches lediglich den Deckel öffnet und den Schalterarm ausfährt. Dabei sollte man sich, von kleinen Winkeln ausgehend, langsam an die nötigen Winkel herantasten, bis der Schalter vom Hebelarm umgelegt wird. Werden die Winkel zu groß gewählt, hört man deutlich, dass die Servomotoren gegen einen zu großen Widerstand versuchen anzusteuern.
 

5.2.3	Setup

Nach der Initialisierung folgt das Setup des Programmcodes.

void setup() {
	pinMode(switchPin, INPUT_PULLUP); 
	handServo.attach(handServoPin); 
	boxServo.attach(boxServoPin); 
	handServo.write(switchDown); 
	boxServo.write(zu); 
}

Als erstes wird der Eingangs-Pin definiert. Diesem wird die Pin-Nummer zugewiesen und er wird als Pull-Up-Pin definiert. (Siehe Kapitel 5.2 bzgl. Pull-Up).

Danach werden den Servomotoren den dazugehörigen Pins zugeordnet und ihnen ein Startwert übergeben.

5.2.4	Das eigentliche Programm

Das eigentliche Programm wird in der Methode loop() ausgeführt. Diese wiederholt sich durchweg, was für die Anwendung von Mikrocontrollern üblich ist.

void loop() {
	…
}

Es wird wiederholt der Zustand des angeschlossenen Schalters ausgelesen und in der Variable switchStatus gespeichert.

switchStatus = digitalRead(buttonPin); 


Der Zustand wird überprüft und sobald der Schalter betätigt wurde (Zustand LOW) wird eine der verschiedenen Methoden ausgeführt, abhängig davon, was in der Variable action hinterlegt ist.

if (switchStatus == LOW) { 
	if (action == 0) {
		…
	}
}

 
Eine einfache Methode sieht hier wie folgt aus.

boxServo.write(auf); 
delay(1000);
	Öffnen des Deckels.

handServo.write(switchOff + 30);
delay(1000); 
	Hand ausfahren, aber den Schalter noch nicht berühren.

handServo.write(switchDown); 
delay(1000);
	Hand wieder in Ausgangsstellung.

for (int i = switchDown; i > switchOff; i--) { 
	handServo.write(i);
	delay(10);
}
	Langsames Ausfahren der Hand.

Hier werden alle Zwischenwinkel einzeln angefahren, um die Geschwindigkeit des Arms kontrollieren zu können. 

for (int i = switchOff; i < switchDown; i++) {
	handServo.write(i);
	delay(10);
}
	Langsames Einfahren der Hand.

handServo.write(switchDown); 
delay(1000); 
	Hand wieder in Ausgangsstellung.

boxServo.write(zu); 
delay(1000);	Schließen des Deckels.

 	WICHTIG!

Da der Servo für die Handsteuerung aus Platzgründen um 180° verdreht zum Servo der Deckelsteuerung eingebaut ist, müssen die Winkel von groß nach klein angesteuert werden. Das bedeutet, dass die Grundstellung des Handservos bei 140° liegt und die Stellung zum Umlegen des Schalters bei 30°. Das ist auch wichtig bei den for-Schleifen, da diese in umgekehrter Richtung zählen müssen. Negativ beim Ausfahren des Arms (i--), positiv beim Einfahren des Arms (i++).


 
5.2.5	Übertragen des Programmcodes auf den Mikrocontroller

Zur Übertragung des Programmcodes auf den Mikrocontroller wird dieser mittels des mitgelieferten USB-Kabels an den Rechner angeschlossen. Nach korrekter Einstellung der Arduino Software kann der Code dann über die entsprechende Schaltfläche (è) übertragen werden. Der Anschluss des USB-Kabels an den Mikrocontroller kann ohne Entfernen des Inlays erfolgen. Hierfür wurde entsprechend eine Aussparung im Inlay gelassen.

Nach erfolgreichem Hochladen des Codes erscheint eine entsprechende Rückmeldung in der Arduino Software.

5.3	Anschluss-Pins

Anschluss-Pins des Arduinos, die mit pinMode(pin, INPUT) konfiguriert sind, stellen nur geringe Anforderungen an die Schaltung, was sie nützlich für das Lesen eines Sensors oder Signals macht.

Hat man den Pin als INPUT konfiguriert und möchte einen Tastenzustand einlesen, befindet sich der Eingangs-Pin im geöffneten Zustand in einem „Schwebezustand“. Um das Signal korrekt auslesen zu können, muss ein Pull-Up- oder Pull-Down-Widerstand verwendet werden. Der Zweck eines solchen Widerstands ist es, den Anschluss in einen definierten Zustand zu „ziehen“, wenn der dieser geöffnet ist. Hierfür wird normalerweise ein Widerstand von etwa 10 Kiloohm gewählt, da er niedrig genug ist, einen schwebenden Zustand des Eingangs zu verhindern und gleichzeitig hoch genug, um  nicht zu viel Strom abzunehmen, wenn der Eingang geschlossen wird.

•	Wird ein Pull-Down-Widerstand verwendet wird, ist die Spannung am Eingangs-Pin bei geöffnetem Eingang LOW und bei geschlossenem Eingang HIGH.
•	Wird ein Pull-Up-Widerstand verwendet, ist die Spannung am Eingangs-Pin bei geöffnetem Eingang HIGH und bei geschlossenem Eingang LOW.

Der auf einem Arduino-Board verwendete ATmega-Mikrocontroller verfügt über interne Pull-Up-Widerstände (Widerstände, die intern an die Stromversorgung angeschlossen werden). Um diese anstelle von externen Pull-Up-Widerständen zu verwenden, können diese mit pinMode(pin, INPUT_PULLUP) eingeschalten werden .

 	ACHTUNG!

Anschlüsse, die mit INPUT oder INPUT_PULLUP als Eingänge konfiguriert sind, können beschädigt oder zerstört werden, wenn sie an negative Spannungen oder an zu hohe Spannungen (>5V oder >3,3V) angeschlossen werden.

 
5.4	Umsetzung auf dem Raspberry Pi

Die Schaltung kann prinzipiell genauso auf einem Raspberry Pi mit GPIO-Board umgesetzt werden. 

import RPi.GPIO as GPIO
import time

buttonPin = 35 # GPIO 19
handServoPin = 11 # GPIO 17
headServoPin = 15 # GPIO 22
    
def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(handServoPin, GPIO.OUT)
    GPIO.setup(headServoPin, GPIO.OUT)
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(buttonPin, GPIO.BOTH, callback=detect, bouncetime=20)


Allerdings stellt die Programmierung der Steuerung in Python eine Herausforderung dar. Im Gegensatz zu Programmierung in Arduino gibt es hier keine Bibliothek für Servomotoren. Diese können zwar angesteuert werden, allerdings nicht mit dem anzufahrenden Winkel, sondern mit Hilfe der Pulsweitenmodulation (PWM). Der Winkel der Servomotoren wird hier über die Dauer des Signals angegeben.

So entspricht eine Pulsdauer von 0,5 ms einem Winkel von 0° und eine Pulsdauer von 2,5 ms einem Winkel von 180°. Mit einer Frequenz von 50 Hz ergibt sich eine Zyklusdauer von 20 ms. 

Somit ergibt sich ein “Duty Cycle“ bei 0° bzw. 0,5 ms von 2,5 % und bei 180° bzw. 2,5 ms von 12,5 %. 

Rechnung:

0,5 ms = 0°
2,5 ms = 180°

Frequenz: 50 Hz / Zyklus: 20 ms.

0° = 0,5 ms / 20 ms = 0.025 = 2.5 %
180° = 2,5 ms / 20 ms = 0.125 = 12,5 %
	180° = 2,0 ms = 10 %
	1° = 2,0 ms / 180° = 

2 ms = 180° -> 1° = 2 ms / 180° = 0.0111… ms



hand.ChangeDutyCycle(get_pwm(90)) # 90 Grad
time.sleep(servo_time(90))


Um die Umrechnung von anzufahrendem Winkel in entsprechenden Duty Cycle zu vereinfachen, kann man dies mithilfe einer Methode vereinfachen.

def get_pwm(angle):
    return (angle/18.0) + 2.5


Außerdem muss nach jedem Befehl zum Anfahren eines Servowinkels eine Pause im Programmcode eingefügt werden, da der Servomotor sonst nicht genug Zeit hat den vorgegebenen Winkel anzufahren, bevor der nächste Befehl ausgeführt wird.

Auch hier kann man sich mit einer Methode behelfen, die berechnet, wie lange der Servomotor für das Anfahren des Winkels benötigt. Aus dem Datenblatt der Servomotoren kann man eine Geschwindigkeit von 0,12 s pro 60° entnehmen. Damit ergibt sich für die Dauer, um ein Grad zu verfahren 0,12 s / 60° = 0,002 s / 1°

def servo_time(degree):
    return (degree/60)*0.12


Zusammengefasst ist eine Umsetzung mithilfe eines Raspberry Pis machbar, allerdings gestaltet sich die Programmierung etwas komplizierter als in Arduino. Des Weiteren ist das Raspberry Pi für den Verwendungszweck überdimensioniert, da man für die Umsetzung weder eine grafische Oberfläche noch WLAN-, Bluetooth- oder andere Schnittstellen benötigt.
 
6	Bedienung

1.	Einschalten der Useless Machine am Batteriefach an der Unterseite.
 

2.	Umlegen des Schalters.
 			 


 
 
7	Kostenaufstellung

Artikel	Artikelnummer	Anzahl	Einzelpreis	Preis	Verkäufer
Elektrische Sparbüchse	-	1	15,99 €	15,99 €	www.amazon.de
Servo SG90 TowerPro	A-5-3	2	2,90 €	5,80 €	www.funduinoshop.com *
Funduino NANO R3 - CH340 Chip - fertig gelötet	Z-7-1	1	4,90 €	4,90 €	www.funduinoshop.com *
Breadboard mit 170 Steckplätzen - div. Farben	F-7-X	1	0,90 €	0,90 €	www.funduinoshop.com *
Kondensator	KT-160	1	0,13 €	0,13 €	www.funduinoshop.com *
Kippschalter (blau) Rocketswitch EIN / EIN Rastend 125V 5A 	KT-21	1	0,69 €	0,69 €	www.funduinoshop.com *
Breadboard Kabel für Arduino	A-4-4	1	3,49 €	3,49 €	www.funduinoshop.com *
Batteriehalter für 4x AA-Batterien/-Akkus mit Ein-Aus-Schalter und losen Kabelenden	-	1	3,99 €	3,99 €	www.amazon.de
AA-Batterien, wiederaufladbar	-	4	-	7,51 €	www.amazon.de
Gesamt				43,40 €	

* 10% Rabatt für Bildungseinrichtungen, Schüler und Studenten.
  
8	Abbildungsverzeichnis

Abbildung Titelseite: Amazon Produktseite

Abbildung 1, Seite 4:  https://store-	cdn.arduino.cc/uni/catalog/product/cache/1/image/1000x750/f8876a31b63532bbba4e781c30024a0a/a/0/a000005_front.jpg 

Abbildung 2, Seite 4: https://content.arduino.cc/assets/Pinout-NANO_latest.png

Abbildung 3, Seite 6: https://www.funduinoshop.com/WebRoot/Store14/Shops/78096195/5539/1F6C/F1E1/642E/B3F3/C0A8/2ABB/B3D1/Servo.png
 
9	Literaturverzeichnis

What is Arduino? (o. D.). Arduino. Abgerufen am 16. März 2021, von 	https://www.arduino.cc/en/Guide/Introduction/

Arduino Nano | Arduino Official Store. (o. D.). Arduino. Abgerufen am 14. März 2021, von 	https://store.arduino.cc/arduino-nano

Arduino-Referenz - Arduino-Referenz. (o. D.). Arduino. Abgerufen am 13. März 2021, von 	https://www.arduino.cc/reference/de/

Grieger, C. (2018, 7. Februar). Servo-Motoren mit Arduino. Elektronik-Projekte und 	Experimente. Abgerufen am 12. März 2021, von 	https://elektro.turanis.de/html/prj036/index.html

INPUT / OUTPUT / INPUT_PULLUP. (o. D.). Wagoi. Abgerufen am 22. März 2021, von 	http://www.wagoi.de/arduino/referenz/variablen/input-output-input_pullup/

Nr.13 Servo ansteuern | Funduino - Kits und Anleitungen für Arduino. (2019, 5. April). 	Abgerufen am 12. März 2021, von Funduino GmbH. https://funduino.de/nr-12-servo-	ansteuern

Overmeire, P., Emmanuel, O., Overmeire, P. & H. (2018, 26. Dezember). Using the SG90 Servo 	Motor With an Arduino. Electronics-Lab.Com. Abgerufen am 12. März 2021, von 	https://www.electronics-lab.com/project/using-sg90-servo-motor-arduino/

Raspberry Pi Servo Motor Steuerung. (2017, 12. Dezember). Raspberry Pi Tutorials. 	Abgerufen am 13. März 2021, von https://tutorials-raspberrypi.de/raspberry-pi-	servo-motor-steuerung/

 
10	Anhänge

Programmcode
useless_machine_v3.ino
useless_machine_v3_random.ino

3D-Modelle
Useless_Machine_Baugruppe.stl
Useless-Machine-Hebel_Gerade.stl
Useless-Machine-Hebel.stl
Useless-Machine-Schalter.stl

