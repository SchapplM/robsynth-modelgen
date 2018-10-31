# Maple-Toolbox: Dynamik für parallele Roboter

Dieser Ordner enthält Maple-Arbeitsblätter zur Generierung von Dynamikfunktionen für (symmetrische) parallele Roboter. Für parallele
Roboter muss die Datei robot_env mit den Definitionen (MDH-Parametern usw.) und robot_env_par mit Informationen zur parallelen Struktur
erstellt werden. Beispiele sind in \robot_codegen_definitions\examples zu finden (Präfix "P" für parallel und "S" für seriell).

## Berechnung mit Bash-Skript

Zur Berechnung müssen die oben berechneten Definitionsdateien vorliegen. Die Nutzung für PKMs erfolgt wie bei seriellen bzw. hybriden
Robotern, es muss allerdings das Argument --parrob übergeben werden, damit das Skript auch parallele Berechnungen mit robot_env_par
durchführt. Wenn die Dynamik der seriellen Ketten bereits berechnet wurde, kann dieser Schritt mit --not_gen_serial übersprungen werden.
Es wird dann auf die Dateien der bereits berechnete Dynamik (save-Dateien aus Maple und .sh-Definitionsdateien aus der Bash) 
zurückgegriffen.

## Manuelle Berechnung:

Führe folgende Arbeitsblätter nacheinander aus, nachdem bereits die inv. Dynamik für ein Bein nach dem bekannten Vorgehen
berechnet wurde:

Normale Form:

```
/robot_codegen_parallel/robot_para_definitions.mw
/robot_codegen_parallel/robot_para_rotmat_kinematics.mw
/robot_codegen_parallel/robot_para_plattform_rotmat_dynamics.mw
/robot_codegen_parallel/robot_para_rotmat_projection_dynamics.mw
```

Regressorform:

```
/robot_codegen_parallel/robot_para_definitions.mw
/robot_codegen_parallel/robot_para_rotmat_kinematics.mw
/robot_codegen_parallel/robot_para_plattform_rotmat_dynamics_regressor.mw
/robot_codegen_parallel/robot_para_rotmat_projection_dynamics_regressor.mw
```