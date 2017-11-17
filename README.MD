# IRT-Maple-Toolbox

Die IRT-Maple-Toolbox enthält Maple-Arbeitsblätter zur Generierung von Dynamikfunktionen für Roboter.
Ausgehend von einer kinematischen Beschreibung werden Dynamikfunktionen in Matlab weitestgehend automatisch generiert.

Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03

(C) Institut für Regelungstechnik, Universität Hannover

## Inhalt

* [Codeerstellung starten](#codeerstellung)
  * [Vorbereitung](#start_vorbereitung)
  * [Alternative 1](#start_alt1)
  * [Alternative 2](#start_alt2)
  * [Kinematische Zwangsbedingungen](#start_zwangsbed)
  * [Einbinden Zusätzlicher Dateien](#start_zusatzdateien)
* [Benennungsschema der Dateien](#benennung)
* [Ordnerstruktur](#ordnerstruktur)


## Kurzversion dieser Anleitung

Folgende Schritte müssen durchgeführt werden:

* [Installation und Einrichtung der Abhängigkeiten](#voraussetzungen)
* [Eingabe der Roboterkinematik in Form der MDH-Parameter](#start_vorbereitung)
* [Starten der Code-Erstellung](#start_alt1)
* [Kopieren der generierten Dateien in eine eigene Ordnerstruktur](#ergebnisordner)

## Voraussetzungen <a name="voraussetzungen"></a> 

* Linux-Betriebssystem (für .sh-Skripte in [Alternative 1](#start_alt1)). Ausführung der Maple-Arbeitsblätter in [Alternative 2](#start_alt2) auch unter Windows möglich.
* Maple muss im Ordner /opt/maple2017/ installiert sein (Standard-Ordner bei sudo-Installation unter Linux).
* Die IRT-Matlab-Toolbox (git@130.75.135.95:software/matlab) muss im Matlab-Pfad initialisiert sein (Aufruf von irt_tb_init.m im Repo). Damit die Pfadinitialisierung auch im Matlab-Terminalaufruf funktioniert, sollte die startup.m richtig gesetzt sein.

## Starten der Codeerstellung <a name="codeerstellung"></a> 

Der vollständige Ablauf zur Generierung von Matlab-Dynamikfunktionen wird in diesem Teilkapitel beschrieben.

### Vorbereitung <a name="start_vorbereitung"></a> 

Die MDH-Parameter (modifizierte Denavit-Hartenberg Parameter nach [KhalilKle1986](#quellen)) müssen in die Umgebungsvariablendatei robot_env im Verzeichnis robot_codegen_definitions eingetragen werden.

        robot_codegen_definitions/robot_env

Diese ist eine Maple-Eingabedatei und definiert Maple-Variablen, die in allen Maple-Arbeitsblättern verwendet werden. Als Vorlage können die Dateien im Verzeichnis robot_codegen_definitions/examples dienen.
Zu beachten ist, dass in die Maple-Vektoren am einfachsten nur symbolische Ausdrücke eingesetzt werden sollten, die so heißen, wie das Vektorelement selbst. Also __a1__, __a2__ in __a__, __d1__, __d2__ in __d__, __qoffset3__, __qoffset4__ in __qoffset__ und so weiter. Zusätzlich können Nullen und Winkelangaben ("__Pi__", großgeschrieben) eingesetzt werden (vereinfacht die Ergebnisse).
Sollen die Geometrieparameter anders benannt sein (z.B. __l1__, __l2__), dann sind die Hinweise aus dem Abschnitt [Werte für Kinematikparameter](#werte_kinematikparameter) zu beachten.
Andere Variablennamen werden bei der Generierung von Matlab-Funktionen nicht erkannt. Beispiel:

        qoffset := Matrix(7,1,[0, -(1/2)*Pi, qoffset3, qoffset4, 0, 0, 0]):

Die Variablen "robot_name" (Präfix für alle Dateien und Funktionsnamen) und "NJ" (Anzahl der Gelenke) müssen an den Roboter angepasst werden.

Für die Variablennamen in robot_env und in [KhalilKle1986](#quellen) gilt folgender Zusammenhang. Die Reihenfolge der Transformationen entspricht der Reihenfolge der Tabellenzeilen.

| [KhalilKle1986](#quellen) | Vorkommnis | robot_env | Beschreibung  | Anmerkung |
|---|---|---|---|---|
| a | 3. Spiegelstrich unter Fig. 6 | v | Vorgänger-Index | Trivial bei seriellen Strukturen (Zahlen von 0 bis NJ-1) |
| gamma | (9) | beta | 1. Rotation um z | Wird nur bei Baumstrukturen benötigt. Bei seriellen Strukturen Null setzen. |
| epsilon | (9) | b | 2. Translation in z-Richtung | Wird nur bei Baumstrukturen benötigt. Bei seriellen Strukturen Null setzen. |
| alpha | (7)  | alpha | 3. Rotation um x |   |
| d | (7) | a | 4. Translation in x-Richtung |   |
| theta | (7) | theta | 5. Rotation um z | Der konstante Offset auf den bei Drehgelenken zeitabhängigen Gelenkwinkel wird so gewählt, dass die Gesamttransformation passt |
| r | (7) | d | 6. Translation in z-Richtung |   |
| sigma | Text unter Fig. 5 | sigma | Art des Gelenks: 0=rotatorisch (revolute, 1dof), 1=translatorisch (prismatic, 1dof) | Für Gelenke mit sigma=1 muss qJ_t in den Parameter d (Verschiebung in z-Richtung) eingetragen werden, für sigma=0 muss qJ_t in theta stehen (Drehung um die z-Achse) |

Nach dem Erstellen dieser Datei werden die Dynamikfunktionen mit Maple generiert, wie im Folgenden beschrieben wird.

### Alternative 1: Automatisches Generieren aller Dateien <a name="start_alt1"></a> 

Mit folgendem Aufruf werden alle Maple-Ausdrücke erzeugt und Matlab-Funktionen exportiert (wie unter Alternative 2 beschrieben).

        ./robot_codegen_start.sh

Folgende Argumente sind möglich:

* Parallele Berechnung der Maple-Arbeitsblätter (schnellere Berechnung):

        -p, --parallel

* Nur Berechnung der Fixed-Base Funktionen (kürzere Berechnung)
 
        --fixb_only

* Nur Berechnung der Floating-Base Funktionen (sinnvoll, falls fixed-Base Modell schon vorhanden ist)
 
        --floatb_only

Zum Speicherort der Ergebnisse siehe [Ergebnisordner](#ergebnisordner).

### Alternative 2: Händisches Generieren aller Dateien <a name="start_alt2"></a> 

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
        /robot_codegen_dynamics/robot_chain_floatb_rotmat_dynamics_regressor.mw

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
        robot_codegen_testfunctions/robot_varpar_fixbase_paramlin_test.m
        robot_codegen_testfunctions/robot_compile_test.m
        robot_codegen_testfunctions/robot_test_everything.m

Beim Durchlaufen der Tests sollten keine Fehler auftreten.

### Systeme mit kinematischen Zwangsbedingungen <a name="start_zwangsbed"></a> 

Neben seriellen und verzweigten kinematischen Strukturen, können auch geschlossene Ketten behandelt werden, wenn die daraus resultierenden kinematischen Zwangsbedingungen vollständig eliminiert werden können.
Die Struktur eines verzweigten Systems wird mit MDH-Parametern in der Datei robot_env eingegeben.
Einige MDH-Parameter sind nun aber keine verallgemeinerten Koordinaten des Systems mehr, sondern z.B. zeitabhängige Winkel, die von den verallgemeinerten Koordinaten abhängen (beschrieben durch die kinematischen Zwangsbedingungen).
Folgende Dateien müssen zusätzlich erzeugt werden, damit die Dynamikgleichungen generiert werden können:

#### Anpassung der Definitionen

Folgende Ausführungen beziehen sich auf die robot_env:

Maple-Variablen wie __theta__ können nun neben den verallgemeinerten Koordinaten __qJ_t__ auch andere Ausdrücke beinhalten.
Diese müssen in zwei zusätzlichen Matrizen __kintmp_t__ und __kintmp_s__ aufgelistet werden.
In __kintmp_t__ stehen zeitabhängige Größen der Ersetzungsvariablen aus __kintmp_s__.
*kintmp* steht dabei für temporäre Kinematikparameter. Diese Parameter werden im Folgenden substituiert.

#### Kinematische Zwangsbedingungen

Ein Maple-Arbeitsblatt mit den kinematischen Zwangsbedingungen muss erstellt werden:

        robot_codegen_constraints/%RN%_kinematic_constraints.mpl
        robot_codegen_constraints/%RN%_kinematic_constraints.mw

In diesem Arbeitsblatt werden die Ausdrücke __kintmp_subsexp__, __kintmp_qs__ und __kintmp_qt__ generiert.
Diese Ausdrücke setzen für die Kinematikzwischengrößen aus __kintmp_s__ Ausdrücke nur in Abhängigkeit konstanter Parameter und der verallgemeinerten Koordinaten ein.
__kintmp_subsexp__ enthält dabei Ausdrücke für Sinus und Cosinus der Parameter aus __kintmp_s__.
__kintmp_qs__ und __kintmp_qt__ enthalten Ausdrücke direkt für __kintmp_s__ und __kintmp_t__.
Die Verwendung von __kintmp_subsexp__ kann rechentechnische günstiger sein, da z.B. in Rotationsmatrizen oft nur Sinus und Cosinus eines MDH-Winkels auftreten, diese werden direkt ersetzt.
Andernfalls wäre die Berechnung Sinus(Arctan(...)) notwendig.

#### Werte für die Kinematische Parameter <a name="werte_kinematikparameter"></a> 

Damit die Testfunktionen funktionieren, müssen konkrete Zahlenwerte für die Kinematikparameter vorgegeben werden.
Die Konstanten müssen in der Datei 

        robot_codegen_constraints/%RN%_kinematic_constraints_matlab.m

eingetragen sein und Variablen initialisieren, die den gleichen Namen haben, wie die in __%RN%_kinematic_constraints.mpl__ verwendeten Konstanten.
Z. B. __l1__, __l2__, ... für konstante Längen.
Falls in der Definitionsdatei robot_env auch bereits andere Konstanten als __a1__, ... definiert wurden sind diese hier auch zu definieren.
Die Parameter können im Allgemeinen nicht zufällig zum Testen gewählt werden, da sonst die kinematischen Zwangsbedingungen verletzt werden.

#### Beispiele

Beispiele für die Implementierung von Zwangsbedingungen finden sich in den Dateien für *KAS5m3* bis *KAS5m6* (Kraftassistenzsystem 3. Arm).

### Weitergehendes Arbeiten

Der Aufruf der Funktionen in den Testskripten kann als Vorlage für weitere Arbeiten verwendet werden.
Zum vereinfachten Aufruf empfiehlt es sich, Aufruffunktionen zu erstellen ("wrapper"), in denen die Parameter (Kinematik und Dynamik) durch Aufruf von Parameterfunktionen vorliegen.

#### Zusätzliche Dateien  <a name="start_zusatzdateien"></a> 

Falls zusätzliche Ausdrücke zu einem System exportiert werden sollen, können dazu entsprechende Skripte in den Ordnern

        robot_codegen_additional/maple
        robot_codegen_additional/scripts
        robot_codegen_additional/templates

abgelegt werden.  
Existiert im Ordner *scripts* eine Datei *%RN%\_maple_additional_worksheet_list_%BASE%*, werden darin enthaltene Verweise auf Maple-Skripte im Ordner *maple* mit der Code-Generierung ausgeführt. *%BASE%* bezeichnet die Basisdarstellungsmethode. Möglich sind hier *twist* oder *eulangrpy*. Siehe dazu den Abschnitt [Modellkennung](#benennung_modell).  
Die Datei *%RN%_matlab_codegen_additional* im Ordner scripts wird bei der Matlab-Funktionsgenerierung mit ausgeführt und kann aus dem generierten Matlab-Code sowie Funktionsvorlagen aus dem Ordner *templates* Matlab-Funktionen erstellen.

Die Verwendung dieser zusätzlicher Dateien ist sinnvoll, um die komplette Code-Generierung zu einem System mit einem Startskript durchführen zu können.
Beispiele sind z.B. Gleichgewichtsfunktionen für zweibeinige Roboter oder eingebaute Feder-Dämpfer-Systeme.


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
* jacobig (Geometrische Jacobi-Matrix: Zusammenhang zwischen translatorischer- und Winkelgeschwindigkeit zu den Geschwindigkeiten der verallgemeinerten Koordinaten)
* jacobigD (Zeitableitung der geometrischen Jacobi-Matrix)
* jacobia (Analytische Jacobi-Matrix: Statt der Winkelgeschwindigkeit besteht im rotatorischen Teil der Bezug zur Zeitableitung der Orientierungsdarstellung (z.B. Zeitableitung der Euler-Winkel)
* jacobiaD (Zeitableitung der analytischen Jacobi-Matrix)

Teil 3: Kennung der Kette

* J ("joint"; Nur Einfluss auf die Gelenke, nicht Einfluss auf die Basis)
* JB ("joint-base"; Einfluss der Basis auf die Gelenke; bei Massenmatrix)
* B ("base"; Nur Einfluss auf die Basis)

Teil 4: Modellkennung  <a name="benennung_modell"></a> 

* fixb (Fixed Base Modell: Orientierung, Geschwindigkeit und Beschleunigung der Basis werden nicht berücksichtigt. Impliziert bei einigen Funktionen "joint")
* floatb_twist  (Floating Base Modell: Geschwindigkeit und Beschleunigung der Basis werden berücksichtigt. Nicht aber die Orientierung)
* floatb_eulangrpy  (Floating Base Modell: Orientierung (EulerXYZ), und die Zeitableitungen der Euler-Winkel werden für die Basis-Geschwindigkeit und -Beschleunigung verwendet.)

Teil 5: Regressorform. 
Um die gewünschte Größe zu erhalten, muss der ausgegebene Regressorvektor bzw. -Matrix mit dem Parametervektor multipliziert werden.

* regmin (Regressor basierend auf Minimalparametern) (siehe [GautierKhalil1990])
* reg2 (Regressor basierend auf Parametersatz 2) (siehe [GautierKhalil1988], [GautierKhalil1990])

Teil 6: Berechnungskennung

* s (Funktion wurde aus symbolisch generiertem Code (sym) aus Maple-Skripten erzeugt. Im Gegenssatz zu "num")
* n (Funktion wurde numerisch z.B. nur in Matlab erzeugt. Z.B. rekursives Newton-Euler in Matlab).
* lag (Es wurde das Lagrange-Verfahren benutzt)
* new (Es wurde das Newton-Euler-Verfahren benutzt)
* CRB (Composite Rigid Body Algorithm aus [Featherstone2008](#quellen). Verfahren zur Berechnung der Massenmatrix)

Teil 7: Parameterkennung
Kennzeichnung, welche Parameter als Eingabe für die Funktion verwendet werden.
* vp1 (Parametersatz 1, variable Parameter)
* vp2 (Parametersatz 2, variable Parameter)
* minpar

Teil 8:  Kennung für variable Parameter

Alle Parameter müssen der Funktion übergeben werden (siehe Funktionsköpfe). Ist in vp1/vp2 bereits enthalten.

* vp

Teil 9: Kompilierungskennung

* mex (Kompilierte Funktionen (mit Matlab-Funktion `Mex_Erstellen`) laufen ungefähr 10 mal schneller)

#### Parameter

* Parametersatz 1 (par1). Masse, Schwerpunkt, Trägheitstensor um den Schwerpunkt ("physical parameters")
* Parametersatz 2 (par2). Masse, Erstes Moment, Trägheitstensor um den Koordinatenursprung ("inertial Parameters")
* Minimalparametersatz (minpar). Neugruppierung des Parametersatzes 2 ("base parameters"; base bezeichnet hier nicht die Basis des Roboters, sondern die Minimaldarstellung der Parameter im mathematischen Sinne)

## Globale Variablen in Maple

Diese Variablen sollten in allen Maple-Arbeitsblättern nicht verwendet werden, da sie aus den Ergebnisdateien anderer Arbeitsblätter geladen werden. Die Variablen werden hauptsächlich im Definitionsskript erstellt (robot_tree_floatb_twist_definitions.mw).

Anzahl Freiheitsgrade (Gesamt, Basis und Gelenke)

        N
        NQB
        NQJ

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

        a, d, alpha, qoffset, b, beta, theta, v, sigma

Dynamik-Parameter

        M, r_i_i_Si, mr_i_i_Si, I_i_Si, I_i_i, PV2_mat, PV2_vec

## Ordner- und Dateistruktur <a name="ordnerstruktur"></a> 

Für Maple-Arbeitsblätter werden die eingehenden Ergebnisdateien mit "<--" und die gespeicherten Ergebnisse mit "-->" angezeigt.
Bei beweglicher Basis werden je nach vorheriger Definition Dateien mit floatb_twist oder floatb_eulangrpy erstellt. Hier werden beispielhaft nur die Dateien mit "twist" genannt.

Dateien:
* .mw Maple-Arbeitsblätter
* .mpl Maple-Text. Enthält die selben Befehle wie die Maple-Arbeitsblätter, aber ohne Formatierung. Wird bei Änderung der Arbeitsblätter angepasst. Diese Dateien sind notwendig, um die Berechnungen mit Skripten aus der Kommandozeile auszuführen.
* ..._maple.m Gespeicherte Maple-Ausdrücke (Maple-internes, komprimiertes Format)
* ..._maple die selben Ausdrücke wie in den _maple.m Dateien, allerdings unkomprimiert (im Maple-Eingabeformat). Dadurch vom Benutzer lesbar, aber bei großen Ausdrücken lange Zeitdauer zum Importieren. Dienen eher der Information des Nutzers.
* ..._matlab.m Exportierter Matlab-Code
* proc_... Maple-Prozedur (Funktionsdefinition als Reintext, Maple-Eingabeformat)

### Ergebnisse der Code-Generierung <a name="ergebnisordner"></a> 

Die Ergebnisse der Code-Generierung werden im Ordner "codeexport" abgelegt.
Für jedes System werden eigene Unterordner für temporäre Maple- und Matlab-Ausdrücke ("tmp"), für fertige Matlab-Funktionen ("matlabfcn") und für Testfunktionen ("testfcn") angelegt.
Die beiden letztgenannten sollten danach z.B. in ein eigenes Repo für das untersuchte System kopiert werden. Es bietet sich an, die Ordner direkt beizubehalten.

        /codeexport/
          %RN%/tmp/
          %RN%/matlabfcn/
          %RN%/testfcn/

Die Ergebnisse der Toolbox können am einfachsten anhand der Modultests im Ordner testfcn überblickt werden.

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

Der Ordner examples enthält Beispiele für die Datei robot_env

        /codegen_definitions
          examples
          robot_env
          robot_tree_base_parameter_transformations.mw
              <-- robot_env
              <-- minimal_parameter_vector_fixb_maple
              <-- minimal_parameter_vector_floatb_maple
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
          robot_tree_rotmat_jacobian_baseframe.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_velocity_worldframe_floatb_twist_maple.m

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
              --> minimal_parameter_vector_fixb_maple
              --> energy_kinetic_fixb_regressor_minpar_maple.m
              --> energy_potential_fixb_regressor_minpar_maple.m
          robot_chain_floatb_rotmat_energy_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_floatb_worldframe_par2_maple.m
              <-- robot_energy_kinetic_floatb_linkframe_par2_maple.m
              --> minimal_parameter_vector_floatb_maple
              --> energy_kinetic_floatb_regressor_minpar_maple.m
              --> energy_potential_floatb_regressor_minpar_maple.m
### Dynamik

        /robot_codegen_dynamics
          robot_tree_floatb_rotmat_dynamics_worldframe_par12.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_floatb_worldframe_par1_maple.m
              <-- robot_energy_kinetic_floatb_worldframe_par1_maple.m
          robot_chain_floatb_rotmat_dynamics_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- energy_kinetic_fixb_regressor_minpar_maple.m
              <-- energy_potential_fixb_regressor_minpar_maple.m


### Kinematische Zwangsbedingungen

Die Skripte müssen vom Benutzer erstellt werden, da jedes System unterschiedlich ist.
Die Skripte müssen mit dem Namen des Roboters anfangen (hier Platzhalter *%RN%*)

        /robot_codegen_constraints
          %RN%_kinematic_constraints.mw

### Zusätzliche Dateien

Die Skripte müssen vom Benutzer erstellt werden, je nach System.
Siehe Beschreibung in [Zusätzliche Dateien](#start_zusatzdateien)

        /robot_codegen_additional/scripts
          %RN%_codegen_matlab_additional_varpar.sh
          %RN%_maple_additional_worksheet_list

### Skripte zur Generierung von Matlabfunktionen

Hauptordner für Skripte

        /robot_codegen_scripts
        
Temporäre Dateien zur Erzeugung von Matlabfunktionen.
Enthält Variablendefinitionen und -deklarationen

        /robot_codegen_scripts/tmp

Textbausteine für Funktionsköpfe:

        /robot_codegen_scripts/tmp_head
          robot_matlabtmp_convert_par2_MPV_fixb.head.m
          robot_matlabtmp_coriolisvec_joint_floatb_par1.head.m
          ...

# Quellen <a name="quellen"></a> 

* [KhalilKle1986] Khalil, W. and Kleinfinger, J.-F.: A new geometric notation for open and closed-loop robots (1986)
* [Featherstone2008] Featherstone, R.: Rigid Body Dynamics Algorithms (2008)
* [GautierKhalil1988] A Direct Determination of Minimum Inertial Parameters of Robots (1988)
* [GautierKhalil1990] Direct Calculation of Minimum Set of Inertial Parameters of Serial Robots (1990)

