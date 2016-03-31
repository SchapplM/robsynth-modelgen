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

* [KhalilKle1987] Khalil, W. and Kleinfinger, J.-F.: Minimum operations and minimum parameters of the dynamic models of tree structure robots (1987)

## Starten der Codeerstellung <a name="codeerstellung"></a> 

Der vollständige Ablauf zur Generierung von Matlab-Dynamikfunktionen wird in diesem Teilkapitel beschrieben.

### Vorbereitung

Die MDH-Parameter (modifizierte Denavit-Hartenberg Parameter nach [KhalilKle1987]) müssen in die Umgebungsvariablendatei eingetragen werden. Als Vorlage können die Dateien in robot_codegen_examples dienen.

        robot_env

### Erstellung von Symbolischen Ausdrücken

Für die Erstellung von Ausdrücken für die Dynamik in expliziter Form (nicht: Regressorform) müssen die Maple-Arbeitsblätter in folgender Reihenfolge gestartet werden:

        /robot_codegen_definitions/robot_tree_floatb_twist_definitions.mw

        /robot_codegen_kinematics/robot_tree_floatb_twist_rotmat_mdh_kinematics.mw
        /robot_codegen_kinematics/robot_tree_floatb_twist_rotmat_kinematics_com_baseframe_par1.mw
        /robot_codegen_kinematics/robot_tree_floatb_twist_rotmat_velocity_baseframe_par1.mw
        /robot_codegen_kinematics/robot_tree_floatb_twist_rotmat_velocity_linkframe.mw

        /robot_codegen_energy/robot_tree_floatb_twist_rotmat_energy_baseframe_par1.mw
        /robot_codegen_energy/robot_tree_floatb_twist_rotmat_energy_baseframe_par2.mw
        /robot_codegen_energy/robot_tree_floatb_twist_rotmat_energy_linkframe_par2.mw

        /robot_codegen_dynamics/robot_tree_floatb_twist_rotmat_dynamics_baseframe_par1.mw

Zur Erstellung der Dynamik mit Parametersatz par2 muss die entsprechende Variable im Kopfteil von robot_tree_floatb_twist_rotmat_dynamics_baseframe_par1.mw geändert werden. Ansonsten müssen keine Eingaben vorgenommen werden. 

Für die Dynamik in Regressorform müssen zusätzlich folgende Maple-Arbeitsblätter ausgeführt werden.
Dies funktioniert momentan nur für serielle Strukturen und feste Basis.

        /robot_codegen_energy/robot_chain_fixb_rotmat_energy_regressor.mw
        /robot_codegen_dynamics/robot_chain_fixb_rotmat_dynamics_regressor.mw

### Generierung von Matlab-Funktionen

Folgendes Shell-Skript erstellt automatisch alle Funktionen. Vorher müssen die Maple-Arbeitsblätter durchgelaufen sein und der Matlab-Code somit exportiert worden sein.

        robot_codegen_scripts/robot_codegen_matlab_varpar.sh



### Testen der Matlab-Funktionen

Testfunktionen werden mit folgendem Skript erstellt (und dabei an den Roboter angepasst):

        robot_codegen_scripts/testfunctions_generate.sh

Die Tests werden durch Ausführen der erstellten Skripte mit Matlab gestartet:

        robot_codegen_testfunctions/robot_varpar_invdyn_test.m
        robot_codegen_testfunctions/robot_varpar_floatbase_test.m
        robot_codegen_testfunctions/robot_varpar_paramlin_test.m
        robot_codegen_testfunctions/robot_compile_test.m

Beim Durchlaufen der Tests sollten keine Fehler auftreten.

### Weitergehendes Arbeiten

Der Aufruf der Funktionen in den Testskripten kann als Vorlage für weitere Arbeiten verwendet werden.
Zum vereinfachten Aufruf empfiehlt es sich, Aufruffunktionen zu erstellen ("wrapper"), in denen die Parameter (Kinematik und Dynamik) durch Aufruf von Parameterfunktionen vorliegen.

## Benennung  <a name="benennung"></a> 

### Benennung der Maple-Dateien

#### Kinematik

Art der kinematischen Kette

        tree/chain

Berechnung der Kinematik

        rotmat/quat

Art der Berücksichtigung der Basis

Valide bei Nichtbetrachtung der Rückwirkung der Dynamik auf die Basis:
Zulässig: Exoskelette, Prothesen
Nicht zulässig: Humanoide Roboter

        floatb_twist (Basis mit Geschwindigkeit im Basis-KS). 

Noch nicht implementiert:

        floatb_quat


### Benennung der Matlab-Dateien

Im Folgenden werden die kryptischen Dateinamen der generierten Matlabfunktionen erklärt.
Die umständlichen Dateinamen sind notwendig, um Funktionen zu unterscheiden, die mit unterschiedlichen Berechnungsmethoden erstellt werden. Falls die Dateinamen zu umständlich werden ist eine Umbenennung der generierten Dateien zu empfehlen.

Die einzelnen Bestandteile des Dateinamens werden durch Unterstriche getrennt.

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

* fixb (Fixed Base Modell: Geschwindigkeit und Beschleunigung der Basis werden nicht berücksichtigt. Impliziert bei einigen Funktionen "joint")
* floatb  (Floating Base Modell: Geschwindigkeit und Beschleunigung der Basis werden berücksichtigt)

Teil 5: Regressorform. 
Um die gewünschte Größe zu erhalten, muss der ausgegebene Regressorvektor bzw. -Matrix mit dem Parametervektor multipliziert werden.

* regmin (Regressor basierend auf Minimalparametern)
* regpar2 (Regressor basierend auf Parametersatz 2)

Teil 6: Berechnungskennung

* sym (Funktion wurde aus symbolische generiertem Code (sym) aus Maple-Skripten erzeugt. Im Gegenssatz zu "num")
* num (Funktion wurde numerisch z.B. nur in Matlab erzeugt. Z.B. rekursives Newton-Euler in Matlab).
* lag (Es wurde das Lagrange-Verfahren benutzt)
* new (Es wurde das Newton-Euler-Verfahren benutzt)

Teil 7: Parameterkennung
Kennzeichnung, welche Parameter als Eingabe für die Funktion verwendet werden.
* par1
* par2
* minpar

Teil 8:  Kennung für variable Parameter

Alle Parameter müssen der Funktion übergeben werden (siehe Funktionsköpfe)

* varpar

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

Dateien:
* .mw Maple-Arbeitsblätter
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
          atlas_limb_floatbase_gen_definitions.mw
          robot_tree_floatbase_gen_definitions.mw
              <-- robot_env
              --> robot_tree_floatb_definitions

### Kinematik

Positionen (Koordinatensysteme, Schwerpunkten), Geschwindigkeiten

        /robot_codegen_kinematics
          robot_tree_floatb_twist_rotmat_mdh_kinematics.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              --> robot_kinematics_rotmat_maple.m
          robot_tree_floatb_twist_rotmat_kinematics_com_par1.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              --> robot_kinematics_com_baseframe_floatb_par1_maple.m
          robot_tree_floatb_twist_rotmat_velocity_baseframe_par1.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_baseframe_floatb_par1_maple.m
              --> robot_velocity_baseframe_floatbase_twist_par1_maple.m
          robot_tree_floatb_twist_rotmat_velocity_linkframe.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              --> robot_velocity_baseframe_floatb_twist_maple.m

### Energie

        /robot_codegen_energy
          robot_tree_floatb_twist_rotmat_energy_baseframe_par1
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_baseframe_floatb_par1_maple.m
              <-- robot_velocity_baseframe_floatbase_twist_par1_maple.m
              --> robot_energy_potential_baseframe_par1_maple.m
              --> robot_energy_kinetic_floatb_baseframe_par1_maple.m
          robot_tree_floatb_twist_rotmat_energy_baseframe_par2.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_kinematics_com_baseframe_floatb_par1_maple.m
              --> robot_energy_potential_baseframe_par2_maple.m
          robot_tree_floatb_twist_rotmat_energy_linkframe_par2.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_kinematics_rotmat_maple.m
              <-- robot_velocity_baseframe_floatb_twist_maple.m
              --> robot_energy_kinetic_floatb_linkframe_par2_maple.m
          robot_chain_fixb_rotmat_energy_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_baseframe_par2_maple.m
              <-- robot_energy_kinetic_floatb_linkframe_par2_maple.m
              --> energy_kinetic_fixb_regressor_minpar_maple.m
              --> energy_potential_fixb_regressor_minpar_maple.m

### Dynamik

        /robot_codegen_dynamics
          robot_tree_floatb_twist_rotmat_dynamics_baseframe_par1.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- robot_energy_potential_baseframe_par1_maple.m
              <-- robot_energy_kinetic_floatb_baseframe_par1_maple.m
          robot_chain_fixb_rotmat_dynamics_regressor.mw
              <-- robot_env
              <-- robot_tree_floatb_definitions
              <-- energy_kinetic_fixb_regressor_minpar_maple.m
              <-- energy_potential_fixb_regressor_minpar_maple.m

### Beispielkonfigurationen

        /robot_codegen_examples
          robot_env_atlas5arm.example
          robot_env_atlas5wbody.example

### Skripte zur Generierung von Matlabfunktionen

Hauptordner für Skripte

        /robot_codegen_scripts
        
Temporäre Dateien zur Erzeugung von Matlabfunktionen.
Enthält Variablendefinitionen und -deklarationen sowie Textbausteine für Funktionsköpfe

        /robot_codegen_scripts/tmp

Funktionsköpfe:

        /robot_codegen_scripts/tmp
          robot_matlabtmp_convert_par2_MPV.head.m
          robot_matlabtmp_coriolisvec_joint_floatb_par1.head.m
          ...
