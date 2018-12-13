#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur parameterlinearen Dynamik und wird von robot_codegen_matlab_varpar_par.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere Matlabfunktionen: Parameterlineare Dynamik PKM"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Minimalparametervektor (Fixed Base)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/minimal_parameter_parrob_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_minimal_parameter_para.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_convert_par2_MPV_para.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_If_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
  # echo "%% Conversion Parameterset 1 -> Parameterset 2" >> $zieldat
  # echo "[mrcges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, m);" >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_If_parallel.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Inverse Dynamik
coordmaple=( actcoord plfcoord)
coordmatlab=( qa pf )
for (( coord=0; coord<=1; coord++ )); do # 0=act joints, 1=platform
  # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
  costrmpl=${coordmaple[$coord]}
  costrmat=${coordmatlab[$coord]}
  
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_reg_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_${costrmat}_reg.m
  cat $head_pfad/robot_matlabtmp_invdynJ_para_${costrmat}_regmin.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_xDDP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
  if [ -f $quelldat ]; then
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_xDDP.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

    cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden. "
  fi
done