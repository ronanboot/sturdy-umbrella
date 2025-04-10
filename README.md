Navigation Autonome dans un Labyrinthe avec TurtleBot<br>
Description<br>
Ce projet simule un robot TurtleBot naviguant de manière autonome dans un environnement de type labyrinthe à l'aide de Gazebo. L'algorithme utilise un processus SLAM (Simultaneous Localization and Mapping) pour générer une carte du labyrinthe, puis applique différents algorithmes de planification de chemin (A*, Dijkstra, Greedy) pour trouver le meilleur chemin vers un objectif. En plus de la planification de chemin classique, un planificateur de mouvement dynamique (DWA - Dynamic Window Approach) est utilisé pour tenir compte des contraintes physiques du robot, telles que la vitesse et l'accélération.<br>

Installation<br>
Prérequis<br>
ROS (Robot Operating System) : Nous recommandons ROS Noetic pour ce projet.<br>
Gazebo : La simulation se fait avec Gazebo.<br>
TurtleBot3 : Le modèle de robot utilisé dans ce projet.<br>
Gmapping : Pour le processus SLAM.<br>

Partie 1 : Simulation dans Gazebo d’un Lieu Réel avec Obstacles<br>
Dans cette première partie, l'objectif est de recréer un environnement, comme un labyrinthe, dans le simulateur Gazebo. <br>

Ce fichier contient la description de l'environnement 3D du labyrinthe<br>

Le fichier laby.launch est utilisé pour lancer la simulation Gazebo avec la configuration du monde défini dans le fichier laby.world<br>

roslaunch your_package laby.launch<br>


Partie 2 : Construction de Carte SLAM avec TurtleBot<br>
Partie 3 : Analyse des algorithmes de navigation<br>
Partie 4 : Navigation Autonome d’une Carte Connue avec TurtleBot<br>
Partie 5 : Implémentation d’une tâche de Motion Planning (Planification de Mouvement)<br>



