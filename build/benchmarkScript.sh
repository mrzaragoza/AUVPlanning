#! /bin/bash


#./AUVPlanning -b test_KPIECE.yaml
#./AUVPlanning -b test_PDST.yaml

# ./AUVPlanning -b test_0.yaml
# ./AUVPlanning -b test_1.yaml
# ./AUVPlanning -b test_2.yaml
# ./AUVPlanning -b test_3.yaml
# ./AUVPlanning -b test_4.yaml
# ./AUVPlanning -b test_5.yaml
# ./AUVPlanning -b test_6.yaml

# ./AUVPlanning -b test_0_SR.yaml
# ./AUVPlanning -b test_1_SR.yaml
# ./AUVPlanning -b test_2_SR.yaml
# ./AUVPlanning -b test_3_SR.yaml
# ./AUVPlanning -b test_4_SR.yaml
# ./AUVPlanning -b test_5_SR.yaml
# ./AUVPlanning -b test_6_SR.yaml

# ./AUVPlanning -b test_0_PD.yaml
# ./AUVPlanning -b test_1_PD.yaml
# ./AUVPlanning -b test_2_PD.yaml
# ./AUVPlanning -b test_3_PD.yaml
# ./AUVPlanning -b test_4_PD.yaml
# ./AUVPlanning -b test_5_PD.yaml
# ./AUVPlanning -b test_6_PD.yaml

for d in d_*.yaml
do 
  echo "    $d"
  ./AUVPlanning -b $d
done

# for r in r_*.yaml
# do 
#   echo "    $r"
#   ./AUVPlanning -b $r
# done

# for k in k_*.yaml
# do 
#   echo "    $k"
#   ./AUVPlanning -b $k
# done

# for p in p_*.yaml
# do 
#   echo "    $p"
#   ./AUVPlanning -b $p
# done

# for e in e_*.yaml
# do 
#   echo "    $e"
#   ./AUVPlanning -b $e
# done