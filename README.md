Navigation Autonome dans un Labyrinthe avec TurtleBot
Description<br>
Ce projet simule un robot TurtleBot naviguant de manière autonome dans un environnement de type labyrinthe à l'aide de Gazebo. L'algorithme utilise un processus SLAM (Simultaneous Localization and Mapping) pour générer une carte du labyrinthe, puis applique différents algorithmes de planification de chemin (A*, Dijkstra, Greedy) pour trouver le meilleur chemin vers un objectif. En plus de la planification de chemin classique, un planificateur de mouvement dynamique (DWA - Dynamic Window Approach) est utilisé pour tenir compte des contraintes physiques du robot, telles que la vitesse et l'accélération.

Installation
Prérequis
ROS (Robot Operating System) : Nous recommandons ROS Noetic pour ce projet.
Gazebo : La simulation se fait avec Gazebo.
TurtleBot3 : Le modèle de robot utilisé dans ce projet.
Gmapping : Pour le processus SLAM.

Partie 1 : Simulation dans Gazebo d’un Lieu Réel avec Obstacles
Partie 2 : Construction de Carte SLAM avec TurtleBot
Partie 3 : Analyse des algorithmes de navigation
Partie 4 : Navigation Autonome d’une Carte Connue avec TurtleBot
Partie 5 : Implémentation d’une tâche de Motion Planning (Planification de Mouvement)



