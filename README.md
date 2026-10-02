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
