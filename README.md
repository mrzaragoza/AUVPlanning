# AUVPlanning

Directorio de código del trabajo final de máster de Guillermo Zaragoza Prous. Este trabajo tiene como fin estudiar los algoritmos de planificación de trayectorias de forma específica para vehículos autónomos submarinos.

## Version
1.0

## Instrucciones de instalación
Para poder utilizar el código, es necesario tener instalada la librería [OMPL.app]. [OMPL] es la librería principal del proyecto, y se utiliza en la versión .app para poder reutilizar parte del código de comprobación de colisiones entre el robot y el entorno.

Una vez esté instalado OMPL.app, comprobar que con éste se han instalado las siguientes librerías:
- [Boost]
- [FCL]
- [ASSIMP]

## Instrucciones de uso
El directorio está dividido en las carpetas /src, /includes y /resources.

Cuando esté todo instalado:

Descargar el código del repositorio
```sh
$ git clone git@github.com:mrzaragoza/AUVPlanning.git
```
Crear una carpeta build
```sh
$ mkdir build
$ cd build
```
Hacer el cmake y compilar
```sh
$ cmake ..
$ make
```
Y ejecutar
```sh
$ ./AUVTorpedoBenchmark
```

[boost]: <http://www.boost.org/>
[fcl]: <https://github.com/flexible-collision-library/fcl>
[assimp]: <http://www.assimp.org/>
[ompl]: <http://ompl.kavrakilab.org>
[OMPL.app]: <http://ompl.kavrakilab.org/download.html>
