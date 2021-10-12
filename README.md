# Collision-Avoidance

Il progetto di collision avoidance si occupa di modificare la traiettoria del robot per poter evitare possibili collisioni, il progetto è realizzato su una VM Ubuntu 18.04 dotata di ROS Melodic. <br />
 <br />
Per testarlo è necessario inserire la cartella collision_avoidance_01 all'interno di un workspace ros e compilarla tramite il comando catkin_make. <br />
Una volta installato si utilizza il comando "rosrun collision_avoidance_01 collision_avoidance_01".
All'interno del repository è stata inserita una mappa di prova, per avviarla si utilizza il comando "rosrun stage_ros stageros cappero_laser_odom_diag_2020-05-06-16-26-03.world". <br />
Partendo da fermo, l'ordine di avvio tra stageros e collision_avoidance_01 è irrilevante. <br />
Successivamente per testarlo si può utilizzare la pubblicazione di cmd_vel tramite linea di comando "rostopic pub -r 10 cmd_vel geometry_msg/Twist -- '[x, 0.0, 0.0]' '[0.0, 0.0, y]'", dove x e y corrispondono alla velocità lineare ed angolare da imprimere al robot. <br />
 <br />
[Prima di tutto è necessario avviare la macchina ROS tramite il comando "roscore", nel caso vi siano problemi nel trovare il workspace e i nodi che vi si trovano all'interno, è utile il comando "source devel/setup.bash" eseguito all'interno del workspace]
