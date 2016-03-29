#!/bin/bash 
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Gelenkwinkel
echo "" > $tmp_pfad/robot_matlabtmp_q.m
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "qJ${i}s = q(${i});" >> $tmp_pfad/robot_matlabtmp_q.m
done

# Gelenkwinkelgeschwindigkeit
echo "" > $tmp_pfad/robot_matlabtmp_qD.m
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "qJD${i}s = qD(${i});" >> $tmp_pfad/robot_matlabtmp_qD.m
done

# Gelenkwinkelbeschleunigung
echo "" > $tmp_pfad/robot_matlabtmp_qDD.m
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "qJDD${i}s = qDD(${i});" >> $tmp_pfad/robot_matlabtmp_qDD.m
done

# Kinematik MDH
echo "" > $tmp_pfad/robot_matlabtmp_par_mdh.m
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "a${i} = a_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "alpha${i} = alpha_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "d${i} = d_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "q_offset${i} = q_offset_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "b${i} = b_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done
for (( i=1; i<=$robot_NJ; i++ ))
do
	echo "beta${i} = beta_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_par_mdh.m
done

# Dynamikparameter
echo "" > $tmp_pfad/robot_matlabtmp_par_m.m
for (( i=1; i<=$robot_NL; i++ ))
do
	echo "M${i} = m_num(${i});" >> $tmp_pfad/robot_matlabtmp_m.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_rcom.m
for (( i=1; i<=$robot_NL; i++ ))
do
	echo "SX${i} = rSges_num_mdh(${i},1);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
	echo "SY${i} = rSges_num_mdh(${i},2);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
	echo "SZ${i} = rSges_num_mdh(${i},3);" >> $tmp_pfad/robot_matlabtmp_par_rcom.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_mrcom.m
for (( i=1; i<=$robot_NL; i++ ))
do
	echo "MX${i} = mrSges_num_mdh(${i},1);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
	echo "MY${i} = mrSges_num_mdh(${i},2);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
	echo "MZ${i} = mrSges_num_mdh(${i},3);" >> $tmp_pfad/robot_matlabtmp_par_mrcom.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_Ic.m
for (( i=1; i<=$robot_NL; i++ ))
do
	echo "XXC${i} = Icges_num_mdh(${i},1);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "XYC${i} = Icges_num_mdh(${i},4);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "XZC${i} = Icges_num_mdh(${i},5);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "YYC${i} = Icges_num_mdh(${i},2);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "YZC${i} = Icges_num_mdh(${i},6);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
	echo "ZZC${i} = Icges_num_mdh(${i},3);" >> $tmp_pfad/robot_matlabtmp_par_Ic.m
done

echo "" > $tmp_pfad/robot_matlabtmp_par_If.m
for (( i=1; i<=$robot_NL; i++ ))
do
	echo "XX${i} = Ifges_num_mdh(${i},1);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "XY${i} = Ifges_num_mdh(${i},4);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "XZ${i} = Ifges_num_mdh(${i},5);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "YY${i} = Ifges_num_mdh(${i},2);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "YZ${i} = Ifges_num_mdh(${i},6);" >> $tmp_pfad/robot_matlabtmp_par_If.m
	echo "ZZ${i} = Ifges_num_mdh(${i},3);" >> $tmp_pfad/robot_matlabtmp_par_If.m
done

# Gravitationsvektor
echo "" > $tmp_pfad/robot_matlabtmp_g.m
for (( i=1; i<=3; i++ ))
do
	echo "g${i} = g_base_mdh(${i});" >> $tmp_pfad/robot_matlabtmp_g.m
done

# Basisgeschwindigkeit und Beschleunigung
echo "" > $tmp_pfad/robot_matlabtmp_VB.m
echo "vxs_base = V_base(1);" >> $tmp_pfad/robot_matlabtmp_VB.m
echo "vys_base = V_base(2);" >> $tmp_pfad/robot_matlabtmp_VB.m
echo "vzs_base = V_base(3);" >> $tmp_pfad/robot_matlabtmp_VB.m
echo "omegaxs_base = V_base(4);" >> $tmp_pfad/robot_matlabtmp_VB.m
echo "omegays_base = V_base(5);" >> $tmp_pfad/robot_matlabtmp_VB.m
echo "omegazs_base = V_base(6);" >> $tmp_pfad/robot_matlabtmp_VB.m

echo "" > $tmp_pfad/robot_matlabtmp_VBD.m
echo "vDxs_base = VD_base(1);" >> $tmp_pfad/robot_matlabtmp_VBD.m
echo "vDys_base = VD_base(2);" >> $tmp_pfad/robot_matlabtmp_VBD.m
echo "vDzs_base = VD_base(3);" >> $tmp_pfad/robot_matlabtmp_VBD.m
echo "omegaDxs_base = VD_base(4);" >> $tmp_pfad/robot_matlabtmp_VBD.m
echo "omegaDys_base = VD_base(5);" >> $tmp_pfad/robot_matlabtmp_VBD.m
echo "omegaDzs_base = VD_base(6);" >> $tmp_pfad/robot_matlabtmp_VBD.m
