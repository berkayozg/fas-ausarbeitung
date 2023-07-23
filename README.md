# FAS, Projekt-E

Willkommen zu unserem CARLA-Fahrerassistenzsystem-Projekt!

## Einführung

Unser Projekt konzentriert sich auf die Entwicklung eines Kollisionsvermeidungssystems in der CARLA-Simulator-Plattform. Ein Kollisionsvermeidungssystem ist ein entscheidendes Fahrerassistenzsystem, das die Sicherheit von Fahrzeugen durch die Verhinderung von Zusammenstößen mit anderen Fahrzeugen oder Hindernissen verbessert. Mit Hilfe von CARLA haben wir die Möglichkeit, realitätsnahe Szenarien zu simulieren und unser Kollisionsvermeidungssystem unter verschiedenen Bedingungen zu testen.

## Implementierte Szenarien 

Unser Kollisionsvermeidungssystem umfasst drei Szenarien, die im Folgenden erläutert werden:

### 1. Forward Collision Warning (Warnung vor Frontalkollision)

Im ersten Szenario haben wir eine Warnung vor Frontalkollisionen implementiert. Das System nutzt Radarsensoren, um die Entfernung zu Fahrzeugen oder Hindernissen vor dem Fahrzeug zu überwachen. Wenn das System erkennt, dass sich das Fahrzeug einem Hindernis zu schnell nähert und eine Kollision droht, wird der Fahrer durch visuelle oder akustische Warnsignale alarmiert. Das Ziel dieses Szenarios ist es, die Aufmerksamkeit des Fahrers zu erhöhen und ihn rechtzeitig auf eine mögliche Kollision aufmerksam zu machen.

### 2. Automatic Emergency Braking (Automatisches Notbremsen)

Im Szenario des "Automatischen Notbremsens" haben wir ein Kollisionsvermeidungssystem implementiert, das darauf abzielt, Unfälle zu verhindern oder zumindest deren Schweregrad zu minimieren. Das System überwacht kontinuierlich die Umgebung des Fahrzeugs mithilfe von Radarsensor und erkennt potenzielle Kollisionsgefahren.

Dieses Szenario demonstriert die Wirksamkeit eines gut abgestimmten Kollisionsvermeidungssystems und seine Fähigkeit, kritische Situationen autonom zu erkennen und angemessen zu reagieren. Durch das "Automatische Notbremsen" kann die Unfallvermeidung verbessert werden und somit die Sicherheit im Straßenverkehr maßgeblich erhöht werden.

### 3. Automatic Obstacle Avoidance (Automatisches Hindernisumfahren)

Das dritte Szenario beinhaltet das automatische Umfahren von Hindernissen. Das System erkennt Hindernisse auf der Fahrstrecke und berechnet automatisch eine alternative Fahrtroute, um das Hindernis sicher zu umfahren. Dieser Assistent hilft dem Fahrer, plötzlich auftretenden Hindernissen oder Gefahrensituationen auszuweichen und mögliche Kollisionen zu verhindern.

## Erforderliche Software

Für die Durchführung des Projekts und die Ausführung der implementierten Szenarien werden folgende Softwarekomponenten benötigt:

`CARLA Simulator`: Der CARLA Simulator ist eine leistungsstarke und flexible Simulationsplattform für autonome Fahrsysteme. Er ermöglicht die realistische Simulation von Verkehrsszenarien, Umgebungen und Fahrzeugen. Der Simulator kann von der offiziellen CARLA-Website heruntergeladen und installiert werden.

`Python 3.7`: Das Projekt wurde in der Programmiersprache Python entwickelt. Stellen Sie sicher, dass Python auf Ihrem System installiert ist. Python kann von der offiziellen Python-Website heruntergeladen und installiert werden.

`Jupyter-Notebook oder -Lab`: Jupyter-Notebook ist eine interaktive Entwicklungsumgebung, die die Arbeit mit Python erleichtert und die Visualisierung von Ergebnissen unterstützt. Es kann mit der Installation von Anaconda oder direkt über Python-Pakete installiert werden.

Pakete: Das Projekt erfordert zusätzliche Python-Pakete, um die Funktionalität zu gewährleisten. Die erforderlichen Pakete sind:

* `carla`: CARLA Python-Paket. Um das Projekt auszuführen, benötigen Sie das `carla` Python-Paket.
* `cv2 (OpenCV)`: Zur Verarbeitung und Analyse von Bildern und Videos.
* `numpy`: Zur effizienten Handhabung von numerischen Berechnungen und Datenarrays.
* `math`: Enthält mathematische Funktionen und Konstanten für die Berechnungen.
* `time`: Zur Zeiterfassung und Verzögerungsfunktionen.
* `glob`: Zur Suche nach Dateipfaden und -namen.
* `os`: Zur Interaktion mit dem Betriebssystem und Verzeichnisoperationen.
* `sys`: Zur Systemspezifischen Funktionalitäten und Interaktion.

Diese Softwarekomponenten sind erforderlich, um das Projekt auszuführen und die implementierten Szenarien erfolgreich zu testen. Stellen Sie sicher, dass Sie alle erforderlichen Pakete `in dem jeweiligen Verzeichnis` installiert haben, bevor Sie das Projekt starten.

## Verwendung

Bitte folgen Sie den unten stehenden Schritten, um das Projekt in Jupyter-Notebook auszuführen:

* Stellen Sie sicher, dass Sie alle erforderlichen Softwarekomponenten gemäß dem Abschnitt "Erforderliche Software" installiert haben.
* Öffnen Sie das Jupyter-Notebook und navigieren Sie zu dem Verzeichnis, in dem sich die Projektdateien befinden.
* Führen Sie die Codeteile in der richtigen Reihenfolge aus. Um sicherzustellen, dass alles reibungslos funktioniert, stellen Sie sicher, dass jeder Codeteil erfolgreich ausgeführt wurde und keine Fehler zurückgegeben hat, bevor Sie zum nächsten Teil übergehen.
* Wenn ein Codeteil Fehler zurückgibt, überprüfen Sie die Fehlermeldung sorgfältig, um mögliche Ursachen zu identifizieren. Stellen Sie sicher, dass die erforderlichen Pakete installiert und alle Abhängigkeiten erfüllt sind.
* Falls ein Codeteil nicht wie erwartet funktioniert, führen Sie den betreffenden Teil erneut aus. Sie können auch den Kernel des Jupyter-Notebooks neu starten und dann alle Codeteile nacheinander ausführen.

Vor jedem Aufruf eines Szenarios ist es wichtig, alle zuvor erstellten Aktoren (Fahrzeuge) zu löschen, um Konflikte oder unerwünschtes Verhalten zu vermeiden. Sie können folgendes verwenden, um die Aktoren zu löschen:
```python
#deleting the actors (cars)
delete_all(actor_list)
```
Diese Funktion löscht alle Elemente in der Liste 'actor_list', die zuvor erstellten Fahrzeuge. Nachdem die Aktoren gelöscht wurden, können Sie die Aktoren erneut erstellen und die nächste Szenariofunktion aufrufen.

```python
# Spawning cars in the middle of the road
# Create an empty list to store the spawned actors
actor_list = []

# Get the blueprint library for Tesla vehicles
my_vehicles = world.get_blueprint_library().filter('*tesla*')
# Define the spawn point for the first vehicle (Tesla)
spawn_point = carla.Transform(
    carla.Location(x=-9.890745, y=-211.247208, z=0.281942),
    carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000)
)

# Attempt to spawn the first vehicle (Tesla) at the defined spawn point
vehicle = world.try_spawn_actor(my_vehicles[1], spawn_point)

# Add the spawned vehicle to the actor_list
actor_list.append(vehicle)

# Get the blueprint library for Audi vehicles
my_vehicles_2 = world.get_blueprint_library().filter('*audi*')

# Define the spawn point for the second vehicle (Audi)
spawn_point_2 = carla.Transform(
    carla.Location(x=-9.890745, y=-240.247208, z=0.281942),
    carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000)
)

# Attempt to spawn the second vehicle (Audi) at the defined spawn point
vehicle_2 = world.try_spawn_actor(my_vehicles_2[1], spawn_point_2)
# Add the spawned vehicle to the actor_list
actor_list.append(vehicle_2)
```




