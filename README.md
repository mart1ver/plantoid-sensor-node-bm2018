# plantoid-sensor-node

Noeud de capteurs plantoid, revoie sous forme de messages OSC les valeurs de deux capteurs sonar, de 8 lectures analogiques ansi que la température et l'humidité, le tout devras étre intégré dans un boitier capable d'encaisser les condition du desert du nevada.

Based on wemos D1.

### Prototype front view
##
![plantoid-sensor-node](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/images/front.jpg)
### Prototype rear view
![plantoid-sensor-node](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/images/back.jpg)

### Schematics
![plantoid-sensor-node](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/shema%20fritzing/plantoid_sch%C3%A9ma.jpg)
## Hardware
1x wemos d1,1x cd4061, 1x restor 10k, dht11, various analog sensors,2x sonar module type HC-SR04
### Final board
![plantoid-sensor-node](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/shema%20fritzing/plantoid_circuit%20imprim%C3%A9.jpg)
## Fritzing board
[here](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/shema%20fritzing/plantoid%20box%20Sketch.fzz)
### Software
I use [OSCDataMonitor](https://github.com/kasperkamperman/OSCDataMonitor) to test the board... more soon...
## Protocole OSC
chaque boitiers renvoie une douzaine de valeurs à ces adresses:
```

plantoid/numero_sculpture(int)/numero_boitier(int)/analog1/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog2/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog3/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog4/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog5/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog6/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog7/valeur(int,min 0 ,max 1024)
plantoid/numero_sculpture(int)/numero_boitier(int)/analog8/valeur(int,min 0 ,max 1024)

plantoid/numero_sculpture(int)/numero_boitier(int)/sonar1/valeur(float,distance en centimetres)
plantoid/numero_sculpture(int)/numero_boitier(int)/sonar2/valeur(float,distance en centimetres)

plantoid/numero_sculpture(int)/numero_boitier(int)/temp/valeur(float,temperature au niveau du boitier)
plantoid/numero_sculpture(int)/numero_boitier(int)/hum/valeur(float,humidité au niveau du boitier)

```

```numero_sculpture``` et ```numero_boitier``` sonts définis dans le  [code de la carte](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/code%20arduino/plantoid-osc-sender.ino), une sculpture peut posseder plusieurs boitiers, les signaux son envoyés au changement d'état des capteurs.

### Arduino code
[here](https://github.com/mart1ver/plantoid-OSC-sender/blob/master/code%20arduino/plantoid-osc-sender.ino)

### Usefull links

https://ledaniel.fr/?page_id=188

https://projects.stwst.at/stwst48/plantoid-by-primavera-de-filippi-david-bovill-vincent-roudaut-and-sara-renaud/
...
