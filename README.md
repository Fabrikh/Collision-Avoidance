# Collision-Avoidance

Il progetto di collision avoidance si occupa di modificare la traiettoria del robot per poter evitare possibili collisioni, il progetto è realizzato su una VM Ubuntu 18.04 dotata di ROS Melodic.

## Installazione

Per installarlo è necessario inserire la repository (o solamente la directory collision_avoidance_01) all'interno della directory source di un workspace ros e compilarla tramite catkin_make
```bash
cd esempio_workspace/src
git clone https://github.com/Fabrikh/Collision-Avoidance.git
cd ..
catkin_make
```

## Testare

Per effettuare i test sono necessari quattro bash separati:
* Uno per avviare il roscore tramite 
```bash
roscore
```

* Uno per avviare lo stageros contenente la mappa, nel caso della mappa inclusa nella repository:
```bash
rosrun stage_ros stageros cappero_laser_odom_diag_2020-05-06-16-26-03.world
```

* Uno per avviare il nodo collision_avoidance tramite comando:
```bash
rosrun collision_avoidance_01 collision_avoidance _sens:=800 _prox:=0.5
```
_sens e _prox sono degli argomenti facoltativi che servono per testare il movimento in situazioni differenti, un'alta sensibilità limiterà poco i movimenti, mentre una bassa dipenderà molto dalla posizione degli ostacoli, è consigliabile una sensibilità nell'ordine delle centinaia, la variabile per la prossimità è invece preferibile nell'ordine delle decine e al suo aumento renderà il movimento più cauto in prossimità di ostacoli.

* Uno per pubblicare i cmd_vel necessari a far muovere il robot:
```bash
rostopic pub -r 10 ca_cmd_vel geometry_msg/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```

Per avviare il test senza problemi è consigliabile inserire i comandi nell'ordine sopra indicato, ma è possibile inserirli in altri ordini purché l'avvio del core ROS rimanga per primo.
