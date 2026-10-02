# match_mur_gui

Gemeinsame PyQt5-Basisoberfläche für die MuR-Roboter. Nach dem Build von ROS 2 Jazzy und dem Colcon-Workspace startet sie mit:

```bash
source /opt/ros/jazzy/setup.bash
source /home/rosmatch/colcon_ws/install/setup.bash
ros2 run match_mur_gui general_mur_gui.py
```

## Ubuntu-Anwendungssuche und Desktop-Icon

Der Installer legt für die Basis-GUI und auf Wunsch für Mocap, Cooperative Handling und OAK eigene Starteinträge und Icons im Stil der MuR-Diagnose an. Er richtet die Verknüpfungen für den aktuellen Benutzer ohne `sudo` ein. Die benötigten ROS-Pakete müssen zuvor im Workspace gebaut sein.

```bash
cd /home/rosmatch/colcon_ws/src/match_mur_gui
python3 scripts/install_gui_desktop.py --app all
```

Danach in der Ubuntu-Anwendungssuche nach „MuR Basis-GUI“, „MuR Mocap“, „MuR Cooperative Handling“ oder „MuR OAK Kamera“ suchen oder die gleichnamigen Desktop-Icons doppelklicken. Der Installer prüft die vorhandenen ROS-Executables, bevor er Dateien anlegt. Jede Verknüpfung lädt beim Start `/opt/ros/jazzy/setup.bash` und das Workspace-Setup; ein Terminal ist nicht nötig. Ohne vorgegebene `ROS_DOMAIN_ID` verwendet sie Domain 62. Sie startet nur die gewählte GUI und keine Roboterhardware.

Mit `--app base` wird nur die Basis-GUI installiert; weitere Werte sind `mocap`, `cooperative` und `oak`. `--workspace PFAD` wählt einen anderen Colcon-Workspace, `--no-desktop-shortcut` erzeugt nur Einträge für die Anwendungssuche. Der Aufruf ist wiederholbar. Bestehende fremde Dateien mit denselben Namen werden nicht überschrieben. Auf GNOME werden Desktop-Verknüpfungen als vertrauenswürdig markiert; falls der Dateimanager dies nicht unterstützt, im Kontextmenü „Starten erlauben“ wählen.

## Home über MoveIt

Home nutzt den laufenden `move_group` des ausgewählten Roboters für Planung und Ausführung. Die benannte Zielpose wird aus dessen SRDF gelesen. Dadurch entfällt eine zusätzliche MoveItPy-Instanz einschließlich ihres bisherigen Absturzes beim Beenden. Die Geschwindigkeitseinstellung skaliert weiterhin die geplante Trajektorie.

Vor Planung und Ausführung muss das UR-External-Control-Programm laufen. Der Hardware-Trajektorien-Proxy überwacht zusätzlich den Programmstatus und begrenzt die Ausführungsdauer. Bei Abbruch, Verbindungsverlust oder Zeitüberschreitung fordert er die Stornierung an und deaktiviert Trajektorien- und Geschwindigkeitsregler. Eine fehlende Bestätigung wird als Fehler protokolliert. Nur nach erfolgreicher Ausführung wird ein zuvor aktiver kartesischer Regler wieder aktiviert. Home selbst hat eine Gesamtlaufzeitgrenze von 70 Sekunden mit anschließender Stornierung; die äußere GUI-Prozessgrenze beträgt 90 Sekunden.

Für Änderungen an diesem Ablauf müssen `match_mur_gui`, `mur_control`, `mur_launch_hardware` und `mur_moveit_config` auf den verwendeten Rechnern aktualisiert und gebaut werden. Bereits laufende Proxy- und MoveIt-Prozesse müssen anschließend neu gestartet werden.
