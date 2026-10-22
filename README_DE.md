# Eingebetteter PI-Drehzahlregler

### STM32F411RE-Firmware mit Python-basierter Streckensimulation

Dieses Projekt implementiert einen PI-Regler auf einem
STM32-Mikrocontroller und untersucht dessen Verhalten anhand einer
virtuellen drehzahlgeregelten Strecke in Python. Im Mittelpunkt stehen
die echtzeitfähige Ausführung, die serielle Kommunikation, Tests zur
Störgrößenunterdrückung und reproduzierbare Messergebnisse.

> **Projektumfang:** Die Strecke wird in Python simuliert. Das Projekt
> steuert derzeit keinen physischen Motor.

## Projektziele

-   Einen zeitdiskreten PI-Regler in der STM32-Firmware implementieren.
-   Die Reglerberechnung periodisch über einen Hardware-Timer-Interrupt
    ausführen.
-   Sollwerte, gemessene Prozesswerte, Reglerparameter und
    Telemetriedaten über UART austauschen.
-   Die Firmware mit einer virtuellen Strecke erster Ordnung zu einem
    geschlossenen Regelkreis verbinden.
-   Das Führungsverhalten und die Reaktion auf konfigurierbare
    Laststörungen bewerten.
-   Messwerte und Zeitinformationen aufzeichnen, damit Versuche
    nachvollziehbar und vergleichbar sind.

## Systemübersicht

``` text
                 USB / Virtueller COM-Port
        ┌─────────────────────────────────────┐
        │                                     │
        ▼                                     │
┌──────────────────────┐    MEAS, Wert        │
│ Python-Simulation    │ ───────────────────► │
│                      │                      │
│ Virtuelle Strecke    │ ◄─────────────────── │
│ Dynamik 1. Ordnung   │  U, Ausgang, TIM2…   │
│ Störgrößenmodell     │                      │
│ CSV / JSON / Plots   │                      │
└──────────────────────┘                      │
                                              │
                                    ┌─────────┴──────────┐
                                    │ STM32 NUCLEO-F411RE│
                                    │                    │
                                    │ UART-Befehlsparser │
                                    │ PI-Regler          │
                                    │ TIM2-Interrupt     │
                                    └────────────────────┘
```

Die Python-Anwendung sendet den aktuellen simulierten Prozesswert an das
Board. Der STM32 berechnet die Stellgröße des PI-Reglers und überträgt
sie über UART zurück. Python verwendet diese Stellgröße, um die
virtuelle Strecke zu aktualisieren. So entsteht ein geschlossener
Regelkreis, bei dem der Regler auf echter Embedded-Hardware ausgeführt
wird, während die Strecke auf dem PC simuliert wird.

## Hardware und Software

### Hardware

-   **Board:** STM32 NUCLEO-F411RE
-   **Debug- und serielle Verbindung:** ST-LINK Virtual COM Port über
    USB
-   **UART:** USART2, typischerweise PA2 (TX) und PA3 (RX)
-   **Timer:** TIM2 zur periodischen Ausführung des Reglers

### Software

-   STM32CubeIDE / STM32 HAL
-   Python 3
-   `pyserial`
-   `matplotlib`

Das Python-Skript verwendet außerdem Module der
Python-Standardbibliothek für CSV- und JSON-Ausgaben, Zeitstempel,
Zeitmessungen und die Berechnung von Kennwerten.

## Aufbau des Regelkreises

-   **Reglertyp:** PI-Regler (Proportional-Integral-Regler)
-   **Nominelles Abtastintervall:** 10 ms (`DT = 0.01 s`, entsprechend
    100 Hz)
-   **Reglerausführung:** TIM2-Period-Elapsed-Interrupt
-   **Reglerausgang:** normierter Wert `u`, der im Python-Streckenmodell
    skaliert wird
-   **Kommunikation:** UART mit 115200 Baud, 8 Datenbits, keiner Parität
    und einem Stoppbit
-   **Einstellbare Verstärkungen:** `kp` und `ki` können beim Start der
    Python-Anwendung eingegeben und an die Firmware übertragen werden.
-   **Einstellbarer Sollwert:** Die Zieldrehzahl wird in U/min angegeben
    und an den STM32 gesendet.

Das Python-Streckenmodell verwendet eine Dynamik erster Ordnung mit
einer einstellbaren Zeitkonstante. Zu den aktuellen Standardwerten in
`main.py` gehören eine Prozesszeitkonstante von 0,5 s und eine maximale
modellierte Drehzahl von 1500 U/min. Diese Werte beschreiben die
virtuelle Strecke und keinen realen Motor.

## Tests mit Laststörungen

Die Simulation unterstützt drei Störungsmodi, die in `main.py`
konfiguriert werden:

  -----------------------------------------------------------------------
###  Modus                               Verhalten
  ----------------------------------- -----------------------------------
  `off`                               Keine Laststörung

  `pulse`                             Eine zeitlich begrenzte Störung ab
                                      dem konfigurierten Störzeitpunkt

  `persistent`                        Eine dauerhafte Störung ab dem
                                      konfigurierten Störzeitpunkt
  -----------------------------------------------------------------------

Wichtige Einstellungen in `main.py`:

``` python
DISTURBANCE_MODE = "persistent"  # "off", "pulse" oder "persistent"
DISTURBANCE_TIME = 5.0
DISTURBANCE_DURATION = 0.1
DISTURBANCE_LEVELS = (200.0, 100.0, 50.0, 25.0, 10.0)
```

`DISTURBANCE_MODE` wird vor dem Start ausgewählt. Das Skript durchläuft
anschließend automatisch die fünf Werte aus `DISTURBANCE_LEVELS`.
Bei `pulse` gilt die konfigurierte Dauer für alle fünf Störungshöhen;
bei `persistent` bleibt jede Störung ab dem Störzeitpunkt bis zum
Simulationsende aktiv. Bei `off` wird ebenfalls eine fünfteilige Reihe
ausgeführt, aber keine Störung auf die Strecke angewendet. In diesem
Modus kennzeichnen die Zahlen in den Laufordnernamen nur die jeweilige
Position in der Reihe.

Die Störung wird im virtuellen Streckenmodell als Drehzahl-Offset auf
den effektiven Zielwert modelliert. Das ist ein vereinfachter
Störeingang zur Untersuchung der Störunterdrückung und kein
drehmomentbasiertes Motor- oder mechanisches Lastmodell.

## Messdaten und erzeugte Ergebnisse

Jeder Start der Anwendung erzeugt unter dem konfigurierten
`RESULTS_DIRECTORY` einen Ordner. Darin erhält
jeder der fünf Läufe einen eigenen Ordner, zum Beispiel
`persistent_200` oder `pulse_25`. Die Anwendung speichert je Lauf:

-   **`measurements.csv`** --- Zeitreihendaten wie Simulationszeit,
    vergangene PC-Zeit, TIM2-Zählerstand, Messwert, Sollwert,
    Reglerausgang, Störung und effektiver Zielwert.
-   **`run.json`** --- Versuchskonfiguration, Status und berechnete
    Kennwerte.
-   **`plot.png`** --- Übersicht über Soll- und Istwert, eine
    Vergrößerung um den Störzeitpunkt, den Reglerausgang und das
    Störungssignal.

Die Auswertung umfasst unter anderem die Einschwingzeit innerhalb eines
Toleranzbands von ±2 %, den Maximalwert, das Überschwingen, den
Mittelwert und die stationäre Regelabweichung während der letzten zwei
Sekunden. Bei aktiver Störung werden zusätzlich die Reaktion während der
ersten 1,5 Sekunden und die maximale absolute Abweichung vom Sollwert
während der ersten Sekunde nach Beginn der Störung erfasst.

Sofern die Firmware entsprechende Telemetriedaten liefert, zeichnet das
Skript auch DWT-Zykluszähler für Timerintervalle auf. Dadurch lässt sich
die Zeitmessung innerhalb des Mikrocontrollers von den am PC erfassten
UART-Empfangsabständen unterscheiden. PC-seitige UART-Abstände enthalten
auch Einflüsse der Kommunikation und der Betriebssystemplanung und
dürfen nicht als exakte Dauer des Hardware-Timer-Interrupts
interpretiert werden.

## Verzeichnisstruktur

``` text
STM32_PI_Controller/
├── python/
│   ├── main.py
│   └── results/
│       └── proportional/
│           └── 0.16/
│               └── <Modus>_<Störungshöhe>/
│                   ├── measurements.csv
│                   ├── run.json
│                   └── plot.png
└── README.md
```

Der Beispielpfad `results/proportional/0.16/` entspricht dem aktuell in
`main.py` eingestellten `RESULTS_DIRECTORY`. Passe diesen Wert an, wenn
du Ergebnisse an einem anderen Ort ablegen möchtest.

`main.c` und die übrigen STM32-Projektdateien werden im
STM32CubeIDE-Projekt verwaltet. Damit das Repository unabhängig
aufgebaut werden kann, sollte das vollständige CubeIDE-Projekt ebenfalls
aufgenommen werden.

## Installation und Inbetriebnahme

### 1. STM32-Firmware vorbereiten

1.  Das STM32CubeIDE-Projekt für das NUCLEO-F411RE öffnen.
2.  Die Firmware kompilieren und auf das Board flashen.
3.  Prüfen, dass USART2 mit 115200 Baud konfiguriert ist und TIM2 im
    Interrupt-Modus gestartet wird.
4.  Das Board per USB verbinden und den vom Betriebssystem zugewiesenen
    virtuellen COM-Port ermitteln.

### 2. Python-Abhängigkeiten installieren

``` bash
python -m pip install pyserial matplotlib
```

### 3. `main.py` konfigurieren

Vor dem Start den seriellen Port, den gewünschten Störungsmodus und
gegebenenfalls die Versuchsparameter prüfen:

``` python
PORT = "COM6"  # An den tatsächlich zugewiesenen Port anpassen
BAUDRATE = 115200

DT = 0.01
SIMULATION_TIME = 12.0
MAX_RPM = 1500.0
PROCESS_TIME_CONSTANT = 0.5
SETPOINT_RPM = 1200.0

DISTURBANCE_MODE = "off"  # "off", "pulse" oder "persistent"
DISTURBANCE_TIME = 5.0
DISTURBANCE_DURATION = 0.1
DISTURBANCE_LEVELS = (200.0, 100.0, 50.0, 25.0, 10.0)
```

Die gezeigten Werte entsprechen den Standardwerten in der
bereitgestellten `main.py`. Passe sie an den jeweiligen Versuch an. Beim
Start fragt das Skript `kp` und `ki` einmal ab; durch Drücken der
Eingabetaste werden die konfigurierten Standardwerte übernommen. Diese
Werte gelten für alle fünf Läufe des Batches.

### 4. Versuch starten

Wenn das Board verbunden und die Firmware gestartet ist:

``` bash
cd python
python main.py
```

Das Skript überträgt die Reglerverstärkungen und den Sollwert, empfängt
die STM32-Telemetrie, aktualisiert die virtuelle Strecke anhand des
gemeldeten Timerintervalls (sofern DWT-Daten verfügbar sind; andernfalls
wird `DT` verwendet) und arbeitet die fünf Störungshöhen nacheinander ab.
CSV, JSON und Plot werden je Lauf gespeichert; Plot-Fenster werden nicht
geöffnet.

## Ergebnisse interpretieren

Eine sinnvolle Versuchsreihe ist:

1.  **Sollwertverfolgung:** Mit `DISTURBANCE_MODE = "off"` starten und
    Anstiegs- bzw. Einschwingverhalten, Überschwingen und stationäre
    Regelabweichung prüfen.
2.  **Impulsstörung:** `pulse` aktivieren und die vorübergehende
    Abweichung sowie die Erholung beobachten.
3.  **Dauerstörung:** `persistent` aktivieren und untersuchen, ob der
    Regler den Sollwert ohne anhaltende Abweichung oder dauerhafte
    Stellgrößensättigung wieder erreicht.
4.  **Verstärkungsvergleich:** Den Versuch mit unterschiedlichen `kp`-
    und `ki`-Werten wiederholen. Für einen objektiven Vergleich
    Sollwert, Streckenparameter und Störungseinstellungen unverändert
    lassen.

Ändere jeweils nur eine Variable und bewahre die erzeugten
Batch-Ordner auf. Innerhalb eines Batches bleiben `kp`, `ki` und der
Störungsmodus gleich; die Störungshöhe wird automatisch variiert. So
lassen sich die Auswirkungen der Änderungen nachvollziehbar vergleichen.

## Aktuelle Einschränkungen

-   Die Strecke basiert auf einem vereinfachten Modell erster Ordnung
    und nicht auf einem physikalischen Motormodell.
-   Ein Motortreiber, Encoder oder realer Drehzahlsensor sind derzeit
    nicht Bestandteil des gezeigten Regelkreises.
-   Die simulierte Laststörung wird als Drehzahl-Offset und nicht als
    physikalisches Lastmoment dargestellt.
-   Die Ergebnisse hängen von den gewählten Streckenparametern und der
    Firmware-Implementierung ab. Sie dürfen nicht als gemessene
    Leistungsdaten eines realen Motors dargestellt werden.
-   Das Python-Skript setzt voraus, dass die Firmware das dokumentierte
    UART-Nachrichtenformat und kompatible Telemetriefelder verwendet.

## Gezeigte Kompetenzen

-   Embedded C und STM32 HAL
-   Hardware-Timer-Interrupts und periodische Echtzeitausführung
-   PI-Regelung und Verhalten geschlossener Regelkreise
-   UART-Protokolldesign und Integration von Firmware und PC-Anwendung
-   Python-basierte Streckensimulation und automatisierte Versuche
-   Messwerterfassung, Visualisierung und Bewertung der Regelgüte
-   Zeitmessung mithilfe von Zyklustelemetrie des Mikrocontrollers

## Mögliche nächste Schritte

-   Automatisierten Vergleich mehrerer `kp`-/`ki`-Kombinationen ergänzen;
    die Störungshöhen werden bereits automatisch durchlaufen.
-   Das Streckenmodell anhand von Messdaten eines realen Motors und
    Encoders validieren.
-   Ein drehmoment- und lastbasiertes Motormodell einführen.
-   Automatisierte Tests für UART-Parsing und Grenzfälle des Reglers
    ergänzen.
-   Die PI-Implementierung, Stellgrößenbegrenzung und
    Anti-Windup-Strategie direkt neben dem Firmware-Quellcode
    dokumentieren.

------------------------------------------------------------------------

**Projektname:** Eingebetteter PI-Drehzahlregler\
**Untertitel:** Echtzeitregelung auf dem STM32F411RE mit
Python-basierter Streckensimulation
