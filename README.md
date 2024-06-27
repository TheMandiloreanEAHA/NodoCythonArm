# NodoCytonArm

A continuación se muestra un nodo en ROS (Robot Operarative System), junto con un archivo de Arduino para estabalecer comunicación con una placa SSC-32.

## Tabla de Contenidos

- [Consideraciones](#consideraciones)
- [Función](#función)
- [NodoROS](#nodoros)
- [NodoArduino](#nodoarduino)
- [Uso](#uso)
- [Ayuda](#ayuda)

## Consideraciones

Este nodo fue programado en C++ para ROS Kinetic. Hay que considerar que para que este nodo funcione, hay que tener un master corriendo en ROS, con el
visualizador de Rviz abierto y cargado modelo URDF correspondiente; para hacer esto, puede crearse un launcher.

El programa en Arduino igualmente fue creado desde ROS, en el IDE de arduino debemos indicar que usaremos una Arduino Mega, de lo contrario, maracrá erro al compilar el código.

## Función

Este nodo fue creado para el control de un brazo Cyton Gamma, el cual se compone de 8 servomotores, los cuales se encuentrar en una placa SSC-32.
El control de este brazo se hace mediante el visualizador Rviz. En este visualizador se invoca un elemento llamado "Joint State Publisher GUI"
Este Joint Sate Publisher muestra una pequeña interfaz con todos los moviminetos de los joints generados en un archivo URDF (modelo 3d del robot),
el cual es invocado en Rviz; estos movimientos se muestran en forma de sliders, de esta manera podemos ver las diferentes posiciones posibles dentro
del rango establecido para el joint. Los jointStates son las articulaciones o los movimientos que puede realizar un robot, en este caso,
son las articulaciones del brazo, los movimientos del visualizador deberían coincidir con los del robot físico.

El Joint State Publisher Gui a parte de mostra la interfaz mencionada, como lo dice su nombre, publica los valores a un tópico en ros, el cual se suele contrar como /JointState
Por ejemplo, si en Rviz movemos un slicer del Joint State Publisher a un valor de 1.76, este valor se estará publicando al tópico /JointState.

¿Y esto mueve al robot?

NO, para mover al robot hay que comunicar ROS con la placa SSC-32. Lamentablemente en mi caso no pude lograr una compatibilidad entre amos sistemas.
Sin embargo, Arduino es compatible con ambos, pero **OJO**, Arduino se comunica con ambos sistemas mediante el puerto Serial y necesitamos un puerto
de salida y otro de entrada, por lo que esto no es posible con un ArduinoUno, para este caso, necesitamos una Arduino MEGA, ya que estas placas permiten
el uso de hasta 4 puertos Seriales.

Esto significa que la Arduino Mega será un mediador entre ROS y la SSC-32. Entonces, en Ros publicamos uns mensaje de tipo JointState a un tópico, el nodo en C++ se sucribirá a ese tópico
y hará las adecuaciones necesarias para mandar los datos recibidos de manera más simplificada a otro tópico(Float64MultiArray), Arduino se suscribe a este nuevo tópico, obtiene los valores publicados y se los manda a la SSC-32, la cual, con los datos recibidos, moverá los motores correspondientes.

Entonces, podemos decir que la funcionalidad principal de estos códigos es:

1. Sucribirse a un tópico de Ros desde un nodo en C++
2. Recibir los valores de un mensaje de tipo JointState y manipular la información contenida en él.
3. En ese mismo nodo, crear otro tópico en el cual publicaremos un mensaje de tipo Float64MultiArray.
4. Sucribirnos desde una Arduino MEGA a un tópico de ROS y recibir su mensaje de tipo Float64MultiArray.
5. Mandar la información adecuada a la SSC-32 por el puerto Serial2 de una Arduino Mega.

## NodoROS

Un nodo un ejecutable que usa ROS para comunicarse con otros nodos. Pero también nos sirve para recibir datos de algún tópico y manipularlos.
**NOTA:** Los nodos deben crearse dentro de un paquete de ROS, dentro de la carpeta "src", en la cual se crean como archivos con extensión .cpp o .py
Ya que son los lenguajes que manjea ROS (C++ y python) para la creación de nodos.

Una vez creado el archivo cpp, hay que volverlo ejecutable, para esto, debemos modificar el archivo cmake que se crea por default al crear nuestro paquete de ROS.

```text
Contenido del Cmake Aquí
```

Una vez hecho esto, explicaré brevemente lo que hace este nodo.
Primero, debemos importar las bibliotecas necesarias para que el nodo funcione.
Después he definido un par de funciones auxiliares:
mapFloat (La cual funciona para mapear un valor dentro de un rango de valores en otro), el uso de esta función es importante y que para mover los motores en la placa SSC32, hay que usar pulsos, que son el rango de movimiento de los motores. Este rango es de 500 (el equivalente a 0°) a 2500 (equivalente a 180°).Entonces esta función es útil para sabes cuanto equivalen cierta cantidad de grados a pulsos.
La otra función es radianesAGrados, la cual, como dice su nombre, convierte radianes a grados. Esta función es útil ya que los datos publicados por el Joint State Publisher están en radianes.

Seguido de esto, definimos las varaibles donde guardaremos los datos recibidos por el Joint State Publisher convertidos en grados. También contamos con los publicadores donde mandaremos los datos en un mensaje de tipo Float64Multiarray, así como se definen los dos arrays que se publicarán en el mensaje antes mencionado.

A continiuación nos encontraremos con la función nodeCallback, la cual se ejecuta una vez que el nodo se sucribe al tópico de _/JointState_
Dentro de la función se establecen los tamaños de los arrays que publicaremos en el mensaje, recuperamos los datos del JointState, los convertimos a grados y los asignamos en sus respectivas variables. Una vez hecho esto, almacenamos los valores de las variables dentro de los arrays que publicaremos, pero antes de eso, debemos mapearlos para almacenar los pulsos y no los grados. Finalmente publicaremos los datos.

Ahora, en el main iniciamos el nodo, así como el subscriptor y los publicadores. Después establecemos la velocidad de publicado, que es de 10 veces por segundo y terminamos con un ciclo que continúa con la publicación de datos.

**NOTA:** Como podrás notar, envíamos los datos a través de 2 tópicos, ¿por qué? Al utilzar un sólo tópico, pparecía haber una perdida de datos en Arduino, por lo que se decidió dividir de esta manera y así funciona correctamente.

## NodoArduino

De igual manera comenzamos con la importación de librerías necesarias y la declaración de variables.
Después vamos con la primer función que es **arm_pose**, la cual se ejecuta una vez suscrito al tópico _//nodeFloatArray_
En esta función recibimos los datos del tópico, y lo guardamos en las raviables, pos teriomente invocamos la función **move**, la cual contiene la sintaxis para comunicarse con la SSC-32 mediante el puerto Serial2. Esta función requiere 3 parámetros, el numero del servo, el pulso y el tiempo, para este caso el tiempo será el mismo para todos, que será de 500 milisegundos y los pulsos serán los valores que recibimos de los tópicos y los valores de los motores van de 0 al 7, siendo 0 el más cercano a la base y 7 el mas lejano.

Después nos encontras con la definición de la función **move**
Seguido de esto, declaramos los suscriptores e indicamos a que tópicos deben suscribirse, así como también indicamos qué funciones deben ejecutarse una vez suscritos.

En el **setup** iniciamos el puerto Serial2, iniciamos el nodo y suscribimos a los suscriptores creados anteriormente. Finalmente en el **loop** escribimos un par de líneas de configuración y eso sería todo.

## Uso

Una vez que hayamos compilado nuestro Work Space en ros (catkin_make) y hayamos cargado el programa a nuestra adruino mega, haremos lo siguiente:

1. Levantaremos el master desde una terminal en ROS o lanzaremos un launcher con Rviz y el urdf cargado, el cual es mi caso ya que es mas eficiente y
   así no tendremos tantas terminales abiertas.

```bash
roslaunch nombreDelPaquete nombreDelLauncher.launch
```

2. En otra terminal, debemos ejecutar el nodo

```bash
rosrun nombreDelPaquete nodo1
```

- Opcional: Si queremos corroborar que nuestro nodo esté mandando los datos desdeados, en otra termianl ejecutamos el siguiente comando:

```bash
rostopic echo /nodeFloatArray
```

3. En otra terminal debemos abrir el puerto rosserial (recuerda que la Arduino MEGA debe estar conectada a la computadora donde estés realizando todo este proceso, así como
   a la placa SSC-32).

```bash
# Por corregir

```

Y listo, con esto tendriamos control de nuestro Cyton Gamma desde Rviz.

## Ayuda

Si no estas familiarizado con lo que es Ros, los tópicos o hasta Rvis; te dejo los siguientes links con lo que podrías adentrarte un poco en lo que es ROS:

- [Introducción a ROS](https://wiki.ros.org/es/ROS/Introduccion)
- [Nodos en ROS](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
- [Tópicos en ROS](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
- [Rviz](https://wiki.ros.org/rviz/Tutorials)
- [Joint State Publisher](https://wiki.ros.org/joint_state_publisher)

Si necesitas o quieres comprender mejor la información mandada desde el puerto Serial2 en Arduino, te porporciono acontinuación el manual de la SSC-32:

- [SSC-32 Manual](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/servo-erector-set-system/ses-electronics/ses-modules/ssc-32/ssc-32-manual/)

```bash
# Clonar el repositorio
git clone https://github.com/TheMandiloreanEAHA/NodoCytonArm
```
