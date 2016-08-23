# IRT-Maple-Toolbox

Die IRT-Maple-Toolbox enthält Maple-Arbeitsblätter zur Generierung von Dynamikfunktionen für Roboter.
Ausgehend von einer kinematischen Beschreibung werden Dynamikfunktionen in Matlab weitestgehend automatisch generiert.

Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03

(c) Institut für Regelungstechnik, Universität Hannover

## Inhalt

* [Codeerstellung starten](#codeerstellung)
* [Benennungsschema der Dateien](#benennung)
* [Ordnerstruktur](#ordnerstruktur)


## Quellen

* [KhalilKle1986] Khalil, W. and Kleinfinger, J.-F.: A new geometric notation for open and closed-loop robots (1986)

## Voraussetzungen

* Linux-Betriebssystem (für .sh-Skripte in Alternative 1). Ausführung der Maple-Arbeitsblätter in Alternative 2 auch unter Windows möglich.
* Maple muss im Ordner /opt/maple18/ installiert sein (Standard-Ordner bei sudo-Installation unter Linux).

## Starten der Codeerstellung <a name="codeerstellung"></a> 

Der vollständige Ablauf zur Generierung von Matlab-Dynamikfunktionen wird in diesem Teilkapitel beschrieben.

### Vorbereitung

Die MDH-Parameter (modifizierte Denavit-Hartenberg Parameter nach [KhalilKle1986]) müssen in die Umgebungsvariablendatei robot_env im Verzeichnis robot_codegen_definitions eingetragen werden.

        robot_codegen_definitions/robot_env

 Diese ist eine Maple-Eingabedatei und definiert Maple-Variablen, die in allen Maple-Arbeitsblättern verwendet werden. Als Vorlage können die Dateien im Verzeichnis robot_codegen_examples dienen.

Für die Variablennamen in robot_env und in [KhalilKle1986] gilt folgender Zusammenhang. Die Reihenfolge der Transformationen entspricht der Reihenfolge der Tabellenzeilen.

| [KhalilKle1986] | Vorkommnis | robot_env | Beschreibung  | Anmerkung |
|---|---|---|---|---|
| a | 3. Spiegelstrich unter Fig. 6 | v | Vorgänger-Index | Trivial bei seriellen Strukturen (Zahlen von 0 bis NJ-1) |
| gamma | (9) | beta | 1. Rotation um z | Wird nur bei Baumstrukturen benötigt. Bei seriellen Strukturen Null setzen. |
| epsilon | (9) | b | 2. Translation in z-Richtung | Wird nur bei Baumstrukturen benötigt |
| alpha | (7)  | alpha | 3. Rotation um x |   |
| d | (7) | a | 4. Translation in x-Richtung |   |
| theta | (7) | qJ+q_offset | 5. Rotation um z | Der konstante Offset auf den zeitabhängigen Gelenkwinkel wird so gewählt, dass die Gesamttransformation passt |
| r | (7) | d | 6. Translation in z-Richtung |   |

Nach dem Erstellen dieser Datei werden die Dynamikfunktionen mit Maple generiert, wie im Folgenden beschrieben wird.

### Alternative 1: Automatisches Generieren aller Dateien

Mit folgendem Aufruf werden alle Maple-Ausdrücke erzeugt und Matlab-Funktionen exportiert (wie unter Alternative 2 beschrieben).

        ./robot_codegen_start.sh

Folgende Argumente sind möglich:

* Parallele Berechnung der Maple-Arbeitsblätter (schnellere Berechnung):

        -p, --parallel

* Nur Berechnung der Fixed-Base Funktionen (kürzere Berechnung)
 
        fixb_only

### Alternative 2: Händisches Generieren aller Dateien

Im Folgenden werden die Einzelbefehle gezeigt, die im Gesamt-Skript aus Alternative 1 ausgeführt werden.

#### Erstellung von Symbolischen Ausdrücken

Für die Initialisierung der Variablendefinitionen müssen je nach Berechnung der Basiskinematik die Definitionen für das Modell "twist" oder das Modell "eulangrpy" gestartet werden


Twist: Die Basisgeschwindigkeit wird als Zeitableitung in den verallgemeinerten Koordinaten aufgenommen. Die Orientierung wird nicht berücksichtigt.
Valide bei Nichtbetrachtung der Rückwirkung der Dynamik auf die Basis.  
Zulässig: Exoskelette, Prothesen, fest montierte Roboter  
Nicht zulässig: Humanoide Roboter  

        /robot_codegen_definitions/robot_tree_floatb_twist_definitions.mw

Die Basisorientierung wird mit Euler-Winkeln in der RPY-Konvention beschrieben (aufeinanderfolgende Drehungen um die mitgedrehte x-, y- und z-Achse).

        /robot_codegen_definitions/robot_tree_floatb_twist_definitions.mw
        /robot_codegen_definitions/robot_tree_floatb_eulangrpy_definitions.mw

Für die Erstellung von Ausdrücken für die Dynamik in expliziter Form (nicht: Regressorform) müssen die Maple-Arbeitsblätter in folgender Reihenfolge gestartet werden:

        /robot_codegen_kinematics/robot_tree_floatb_rotmat_mdh_kinematics.mw
        /robot_codegen_kinematics/robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mw
        /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_worldframe_par1.mw
        /robot_codegen_kinematics/robot_tree_floatb_rotmat_velocity_linkframe.mw

        /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par1.mw
        /robot_codegen_energy/robot_tree_floatb_rotmat_energy_worldframe_par2.mw
        /robot_codegen_energy/robot_tree_floatb_rotmat_energy_linkframe_par2.mw

        /robot_codegen_dynamics/robot_tree_floatb_rotmat_dynamics_worldframe_par12.mw

Zur Erstellung der Dynamik mit Parametersatz par2 muss die entsprechende Variable im Kopfteil von robot_tree_floatb_rotmat_dynamics_worldframe_par12.mw geändert werden. Ansonsten müssen keine Eingaben vorgenommen werden. 

Falls der Matlab-Export zu lange dauert, kann der Optimierungsgrad des Code-Exports reduziert werden, indem die Variable codegen_opt auf 1 anstatt auf 2 gesetzt wird (sinnvoll ab Gelenkzahl 20).

Für die Dynamik in Regressorform müssen zusätzlich folgende Maple-Arbeitsblätter ausgeführt werden.
Dies funktioniert momentan nur für serielle Strukturen und feste Basis.

        /robot_codegen_energy/robot_chain_fixb_rotmat_energy_regressor.mw
        /robot_codegen_dynamics/robot_chain_fixb_rotmat_dynamics_regressor.mw

#### Generierung von Matlab-Funktionen

Folgendes Shell-Skript erstellt automatisch alle Funktionen. Vorher müssen die Maple-Arbeitsblätter durchgelaufen sein und der Matlab-Code somit exportiert worden sein.

        robot_codegen_scripts/robot_codegen_matlab_varpar.sh



#### Generieren der Testfunktionen

Testfunktionen werden mit folgendem Skript erstellt (und dabei an den Roboter angepasst):

        robot_codegen_scripts/testfunctions_generate.sh

### Testen der Matlab-Funktionen

Die Tests werden durch Ausführen der erstellten Skripte mit Matlab gestartet:
        
        robot_codegen_testfunctions/robot_varpar_kinematics_test.m
        robot_codegen_testfunctions/robot_varpar_invdyn_test.m
        robot_codegen_testfunctions/robot_varpar_floatbase_test.m
        robot_codegen_testfunctions/robot_varpar_simulink_test.m
        robot_codegen_testfunctions/robot_varpar_paramlin_test.m
        robot_codegen_testfunctions/robot_compile_test.m
        robot_codegen_testfunctions/robot_test_everything.m

Beim Durchlaufen der Tests sollten keine Fehler auftreten.

### Weitergehendes Arbeiten

Der Aufruf der Funktionen in den Testskripten kann als Vorlage für weitere Arbeiten verwendet werden.
Zum vereinfachten Aufruf empfiehlt es sich, Aufruffunktionen zu erstellen ("wrapper"), in denen die Parameter (Kinematik und Dynamik) durch Aufruf von Parameterfunktionen vorliegen.

## Benennung  <a name="benennung"></a> 

Die einzelnen Bestandteile des Dateinamens werden durch Unterstriche getrennt.

### Benennung der Maple-Dateien

Allgemeines Präfix: Code-Erstellung für beliebige Roboter

        robot

Art der kinematischen Kette

        tree/chain

Art der Berücksichtigung der Basis

        floatb/fixb

Berechnung der Kinematik

        rotmat/quat

Ergebnis des Arbeitsblattes

        mdh_kinematics, velocity_linkframe, dynamics_worldframe, ...

### Benennung der Matlab-Dateien

Im Folgenden werden die kryptischen Dateinamen der generierten Matlabfunktionen erklärt.
Die umständlichen Dateinamen sind notwendig, um Funktionen zu unterscheiden, die mit unterschiedlichen Berechnungsmethoden erstellt werden. Falls die Dateinamen zu umständlich werden ist eine Umbenennung der generierten Dateien zu empfehlen.
Die kryptischen Abkürzungen sind für die Beschränkung der Funktionsnamenslänge auf 63 Zeichen notwendig.

Teil 1: Name des Roboters. Wird in Umgebungsvariablendatei robot_env festgelegt

Teil 2: Funktionskennung

* coriolismat (Matrix der Coriolis- und Zentrifugalkraft)
* coriolisvec (Gelenkmomente der Coriolis- und Zentrifugalkraft)
* energypot (Potentielle Energie)
* energykin (Kinetische Energie)
* gravload (Gelenkdrehmoment resultierend aus Gravitationsbelastung)
* inertia (Massenmatrix)
* inertiaD (Zeitableitung der Massenmatrix)
* invdyn (Inverse Dynamik: Gelenkdrehmoment resultierend aus allen dynamischen Einflüssen)

Teil 3: Kennung der Kette

* joint (Nur Einfluss auf die Gelenke, nicht Einfluss auf die Basis)
* joint_base (Einfluss der Basis auf die Gelenke; bei Massenmatrix)
* base (Nur Einfluss auf die Basis)

Teil 4: Modellkennung

* fixb (Fixed Base Modell: Orientierung, Geschwindigkeit und Beschleunigung der Basis werden nicht berücksichtigt. Impliziert bei einigen Funktionen "joint")
* floatb_twist  (Floating Base Modell: Geschwindigkeit und Beschleunigung der Basis werden berücksichtigt. Nicht aber die Orientierung)
* floatb_eulangrpy  (Floating Base Modell: Orientierung (EulerXYZ), und die Zeitableitungen werden für die Basis-Geschwindigkeit und -Beschleunigung verwendet.)

Teil 5: Regressorform. 
Um die gewünschte Größe zu erhalten, muss der ausgegebene Regressorvektor bzw. -Matrix mit dem Parametervektor multipliziert werden.

* regmin (Regressor basierend auf Minimalparametern)
* regpar2 (Regressor basierend auf Parametersatz 2)

Teil 6: Berechnungskennung

* s (Funktion wurde aus symbolisch generiertem Code (sym) aus Maple-Skripten erzeugt. Im Gegenssatz zu "num")
* n (Funktion wurde numerisch z.B. nur in Matlab erzeugt. Z.B. rekursives Newton-Euler in Matlab).
* lag (Es wurde das Lagrange-Verfahren benutzt)
* new (Es wurde das Newton-Euler-Verfahren benutzt)

Teil 7: Parameterkennung
Kennzeichnung, welche Parameter als Eingabe für die Funktion verwendet werden.
* vp1 (Parametersatz 1, variable Parameter)
* vp2 (Parametersatz 2, variable Parameter)
* minpar

Teil 8:  Kennung für variable Parameter

Alle Parameter müssen der Funktion übergeben werden (siehe Funktionsköpfe). Ist in vp1/vp2 bereits enthalten.

* vp

Teil 9: Kompilierungskennung

* mex (Kompilierte Funktionen (mit Mex_Erstellen) laufen bis zu 10 mal schneller)

#### Parameter

* Parametersatz 1 (par1). Masse, Schwerpunkt, Trägheitstensor um den Schwerpunkt ("physical parameters")
* Parametersatz 2 (par2). Masse, Erstes Moment, Trägheitstensor um den Koordinatenursprung ("inertial Parameters")
* Minimalparametersatz (minpar). Neugruppierung des Parametersatzes 2 ("Minimal parameters")

## Globale Variablen in Maple

Diese Variablen sollten in allen Maple-Arbeitsblättern nicht verwendet werden, da sie aus den Ergebnisdateien anderer Arbeitsblätter geladen werden. Die Variablen werden hauptsächlich im Definitionsskript erstellt (robot_tree_floatb_twist_definitions.mw).

Anzahl Freiheitsgrade (Gesamt, Basis und Gelenke)

        N
        NB
        NJ

Anzahl der Körper

        NL

Verallgemeinerte Koordinaten

        q_s, q_t
        qD_s, qD_t
        qDD_s, qDD_t

Verallgemeinerte Koordinaten der Gelenke

        qJ_s, qJ_t
        qJD_s, qJD_t
        qJDD_s, qJDD_t

Verallgemeinerte Koordinaten der Basis

        X_base_t, X_base_s
        V_base_t, V_base_s
        VD_base_t, VD_base_s

MDH-Parameter

        a, d, alpha, qoffset, b, beta, theta, v

Dynamik-Parameter

        M, r_i_i_Si, mr_i_i_Si, I_i_Si, I_i_i, PV2_mat, PV2_vec

## Ordner- und Dateistruktur <a name="ordnerstruktur"></a> 

Für Maple-Arbeitsblätter werden die eingehenden Ergebnisdateien mit "<--" und die gespeicherten Ergebnisse mit "-->" angezeigt.
Bei beweglicher Basis werden je nach vorheriger Definition Dateien mit floatb_twist oder floatb_eulangrpy erstellt. Hier werden beispielhaft nur die Dateien mit "twist" genannt.

Dateien:
* .mw Maple-Arbeitsblätter
* .mpl Maple-Text. Enthält die selben Befehle wie die Maple-Arbeitsblätter, aber ohne Formatierung. Wird bei Änderung der Arbeitsblätter angepasst.
* maple.m Gespeicherte Maple-Ausdrücke (Maple-internes Format)
* matlab.m Exportierter Matlab-Code
* proc_... Maple-Prozedur (Reintext, Maple-Eingabeformat)

### Koordinatentransformation

        /transformation
          proc_rotx
          proc_roty
          proc_rotz
          proc_trafo_mdh
          proc_transl
          proc_trotx
          proc_troty
          proc_trotz

### Hilfsfunktionen

        /helper
          proc_convert_s_t
          proc_convert_t_s
          transformation_print_parameter_names.m
          proc_MatlabExport
          proc_Lagrange1
          proc_LagrangeN

### Definitionen

        /codegen_definitions
          robot_env
          robot_tree_floatbase_twist_definitions.mw
              <-- robot_env
              --> robot_tree_floatb_definitions
              --> robot_tree_floatb_twist_definitions
          robot_tree_floatbase_eulangrpy_definitions.mw
              <-- robot_env
              <-- robot_tree_floatb_twist_definitions
              --> robot_tree_floatb_definitions
              --> robot_tree_floatb_eulangrpy_definitions

### Kinematik

Positionen (Koordinatensysteme, Schwerpunkten), Geschwindigkeiten

        /robot_codegen_kinematics
          robot_tree_floatb_rotmat_mdh_kinematics.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              --> robot_kinematics_rotmat_maple.m
          robot_tree_floatb_rotmat_kinematics_com_par1.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              --> robot_kinematics_com_worldframe_floatb_par1_maple.m
          robot_tree_floatb_rotmat_velocity_worldframe_par1.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_worldframe_floatb_par1_maple.m
              --> robot_velocity_worldframe_floatbase_twist_par1_maple.m
          robot_tree_floatb_twist_rotmat_velocity_linkframe.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              --> robot_velocity_worldframe_floatb_twist_maple.m

### Energie

        /robot_codegen_energy
          robot_tree_floatb_rotmat_energy_worldframe_par1
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_worldframe_floatb_par1_maple.m
              <-- robot_velocity_worldframe_floatbase_twist_par1_maple.m
              --> robot_energy_potential_floatb_worldframe_par1_maple.m
              --> robot_energy_kinetic_floatb_worldframe_par1_maple.m
          robot_tree_floatb_rotmat_energy_worldframe_par2.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_worldframe_floatb_par1_maple.m
              --> robot_energy_potential_floatb_worldframe_par2_maple.m
          robot_tree_floatb_rotmat_energy_linkframe_par2.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_velocity_worldframe_floatb_twist_maple.m
              --> robot_energy_kinetic_floatb_linkframe_par2_maple.m
          robot_chain_fixb_rotmat_energy_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_floatb_worldframe_par2_maple.m
              <-- robot_energy_kinetic_floatb_linkframe_par2_maple.m
              --> energy_kinetic_fixb_regressor_minpar_maple.m
              --> energy_potential_fixb_regressor_minpar_maple.m

### Dynamik

        /robot_codegen_dynamics
          robot_tree_floatb_rotmat_dynamics_worldframe_par12.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_floatb_worldframe_par1_maple.m
              <-- robot_energy_kinetic_floatb_worldframe_par1_maple.m
          robot_chain_fixb_rotmat_dynamics_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- energy_kinetic_fixb_regressor_minpar_maple.m
              <-- energy_potential_fixb_regressor_minpar_maple.m

### Beispielkonfigurationen

        /robot_codegen_examples
          robot_env_atlas5arm.example
          robot_env_atlas5wbody.example
          robot_env_atlas4leg.example
          robot_env_prothese2dof.example
          robot_env_rigidbody.example

### Skripte zur Generierung von Matlabfunktionen

Hauptordner für Skripte

        /robot_codegen_scripts
        
Temporäre Dateien zur Erzeugung von Matlabfunktionen.
Enthält Variablendefinitionen und -deklarationen

        /robot_codegen_scripts/tmp

Textbausteine für Funktionsköpfe:

        /robot_codegen_scripts/tmp_head
          robot_matlabtmp_convert_par2_MPV.head.m
          robot_matlabtmp_coriolisvec_joint_floatb_par1.head.m
          ...
