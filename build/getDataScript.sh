#! /bin/bash

echo "Tipo: $1"
mkdir $1_Data
cwd=$(pwd)
cd $1
echo "++Entra en $(pwd)"

for p in ./*
do 

  cd $p
  echo "++++Entra en $(pwd)"

  rm *~
  logFile=$(ls . | grep *.log)
  echo "    $logFile"
  name=${logFile::-4}
  echo "    $name"
  
  ompl_benchmark_statistics.py $logFile -d $name.db
  ompl_benchmark_statistics.py -d $name.db -p $name.pdf -a
  cp $name.pdf $cwd/$1_Data

  cd ..
  echo "----Vuelve a $(pwd)"
done

cd ..
echo "--Vuelve a $(pwd)"



