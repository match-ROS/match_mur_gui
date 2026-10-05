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

## Ausgewählte URs kontrolliert herunterfahren

Im Tab **UR** den Knopf **Ausgewählte URs herunterfahren** verwenden. Ziel sind ausschließlich die ausdrücklich markierten MuRs und die markierten Armseiten `UR10_l` / `UR10_r`. Ohne MuR- oder Armauswahl wird nichts abgeschaltet. Der Bestätigungsdialog zeigt die konkrete Zielliste; Abbrechen ist voreingestellt. Werkstücke vorher ablegen oder sichern: Der Helfer fährt keine Parkpose an und schaltet keine Greifer gezielt um.

UR beschreibt zuerst das Ausschalten des Arms und anschließend das Herunterfahren der Control Box. Der Helfer setzt dies über den Dashboard Server auf Port 29999 um:

1. PolyScope-Version und bei e-Series den Remote-Control-Modus prüfen.
2. `stop` senden und auf `programState = STOPPED` warten.
3. `power off` senden und auf `robotmode = POWER_OFF` warten.
4. `shutdown` senden und die Antwort `Shutting down` auswerten.

Ein bereits ausgeschalteter Arm wird nach Prüfung von `STOPPED` direkt heruntergefahren. Fehler oder Zeitüberschreitungen verhindern den nächsten Schritt für diesen Arm. Die anderen ausgewählten URs werden unabhängig bearbeitet; Ergebnis und Dashboard-Antworten erscheinen im GUI-Log und im Armstatus. **„Herunterfahren vom UR bestätigt“ bedeutet, dass der Dashboard Server den Befehl angenommen hat**; der endgültige stromlose Zustand wird damit nicht gemessen. Bei Verbindungsverlust ohne Antwort wird kein Erfolg angenommen und `shutdown` nicht automatisch wiederholt.

Die GUI beendet ausgewählte Home-/Align-Abläufe, fordert deren Stornierung an, setzt Jog-Befehle auf null und sperrt Freedrive-Keepalives sowie neue Bewegungsanforderungen für die Zielarme. Verzögerte Enable-Retries werden verworfen. Die Sperre bleibt auch bei Fehlern bestehen, bis ausdrücklich **Enable URs / Ready** oder **Start Hardware** verwendet wird. Laufende Enable-/Start-Preflights und die automatische UR-Aktivierung beim Hardware-Start müssen vor dem Abschalten abgeschlossen sein. Bei fehlgeschlagenem Hardware-Start zuerst **Stop Managed Processes** verwenden. Während des Abschaltens bleibt die GUI offen und „Stop Managed Processes“ lässt den Abschalthelfer weiterlaufen. Andere Bediengeräte und externe Automatisierungen dürfen währenddessen keine Startbefehle senden.

Voraussetzungen: SSH-Zugang zum ausgewählten MuR-Rechner; dessen Hostnamen `UR10_l` und `UR10_r` müssen auf die eigenen URs zeigen. Der Helfer läuft über `python3` direkt aus `Remote WS/src/match_mur_gui/scripts/shutdown_urs.py`, auch ohne laufenden ROS-Treiber. Unterstützt werden CB3 / PolyScope 3 und e-Series / PolyScope 5 ab 5.6 (für die Remote-Abfrage). PolyScope X und unbekannte Versionen werden abgewiesen. `match_mur_gui` auf GUI- und MuR-Rechnern aktualisieren, bauen und die GUI neu starten:

```bash
colcon build --packages-select match_mur_gui --symlink-install
```

MiR, Hubachsen und MuR-Rechner werden nicht heruntergefahren. Das erneute Einschalten der UR-Control-Box erfolgt lokal. Vor Trennen der Netzversorgung das vollständige Herunterfahren abwarten; nach dem Trennen nennt UR 30 Sekunden zum Entladen gespeicherter Energie.

Herstellerquellen:

- [UR: Power Down The Robot](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR5e/H_g5_sections/mechanical_interface_g5/powerdown.htm)
- [UR: Dashboard-Befehle für e-Series](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm)
- [UR: Dashboard-Befehle für CB-Series](https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/)
