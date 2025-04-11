# Navigation Autonome dans un Labyrinthe avec TurtleBot<br>
*Description*<br>
Ce projet simule un robot TurtleBot naviguant de manière autonome dans un environnement de type labyrinthe à l'aide de Gazebo. L'algorithme utilise un processus SLAM (Simultaneous Localization and Mapping) pour générer une carte du labyrinthe, puis applique différents algorithmes de planification de chemin (A*, Dijkstra, Greedy) pour trouver le meilleur chemin vers un objectif. En plus de la planification de chemin classique, un planificateur de mouvement dynamique (DWA - Dynamic Window Approach) est utilisé pour tenir compte des contraintes physiques du robot, telles que la vitesse et l'accélération.<br>

## Installation<br>
*Prérequis*<br>
ROS (Robot Operating System) : Nous recommandons ROS Noetic pour ce projet.<br>
Gazebo : La simulation se fait avec Gazebo.<br>
TurtleBot3 : Le modèle de robot utilisé dans ce projet.<br>
Gmapping : Pour le processus SLAM.<br>

## Partie 1 : Simulation dans Gazebo d’un Lieu Réel avec Obstacles<br>

Dans cette première partie, l'objectif est de recréer un environnement, comme un labyrinthe, dans le simulateur Gazebo. <br>

Le fichier *laby.world* contient la description de l'environnement 3D du labyrinthe<br>

Le fichier *laby.launch* est utilisé pour lancer la simulation Gazebo avec la configuration du monde défini dans le fichier *laby.world*<br>

Dans un terminal : `roslaunch robotique_projet laby.launch`<br>


## Partie 2 : Construction de Carte SLAM avec TurtleBot<br>

Lancez le TurtleBot dans un environnement contrôlé.<br>
Dans un terminal : `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`<br>

Déplacez le TurtleBot dans l’environnement pour couvrir toutes les zones accessibles. Vous peuvent choisir entre implémenter un contrôle par clavier ou viaun joystick externe.<br>
- version keyboard<br>
Dans un terminal : `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`<br>
- version joystick(à vérifier)<br>
Dans un terminal : `roslaunch teleop_twist_joy teleop.launch`<br>

Sauvegardez la carte générée
Dans un terminal : `rosrun map_server map_saver -f ~/map`<br>

## Partie 3 : Analyse des algorithmes de navigation<br>

=== Analyse des Algorithmes de Navigation ===

### ==== Algorithme A* ====
**Description :**<br>
L'algorithme A* est un algorithme de recherche de chemin optimal qui utilise à la fois une
fonction heuristique (h) et le coût total pour atteindre un objectif. Il combine les avantages
des algorithmes de recherche en largeur et best-first.

**Paramètres :**
  - Fonction heuristique (h) : Estimation du coût restant pour atteindre l'objectif.
    Exemple : Distance Euclidienne ou Manhattan.
  - Coût de déplacement (g) : Le coût accumulé pour atteindre un point particulier.
  - Fonction de coût total (f) : f(n) = g(n) + h(n), utilisée pour guider la recherche.

**Analyse :**
  - Une bonne heuristique permet de réduire le temps de calcul en dirigeant la recherche.
  - L'algorithme est optimal si la heuristique est admissible (ne surestime jamais le coût restant).
  - Le choix de la heuristique a un impact majeur sur l'efficacité et la rapidité de l'algorithme.
  - A* est performant pour les environnements avec des obstacles fixes.
  - Cependant, A* peut être plus lent si l'heuristique est mal définie.


### ==== Algorithme de Dijkstra ====
**Description :**<br>
L'algorithme de Dijkstra est un algorithme utilisé pour trouver le chemin le plus court dans un
graphe pondéré. Contrairement à A*, il n'utilise pas de fonction heuristique et explore tous les
chemins possibles jusqu'à ce que la solution soit trouvée.

**Paramètres :**
  - Coût de déplacement (g) : Le coût associé à chaque déplacement entre nœuds.
  - Taux de résolution : La taille de la grille de recherche (la densité des nœuds).
  - Critères d'arrêt : L'algorithme s'arrête dès que le nœud objectif est atteint.

**Analyse :**
  - Dijkstra est garanti de trouver le chemin optimal à condition que tous les coûts soient positifs.
  - La complexité de Dijkstra est O(E log V), ce qui peut être plus lent que A* pour des grilles larges.
  - L'algorithme peut être inefficace dans des environnements avec des heuristiques bien définies.


### ==== Algorithme Greedy Best-First Search (GBFS) ====
**Description :**<br>
L'algorithme Greedy Best-First Search guide la recherche vers l'objectif en utilisant une fonction
heuristique pour estimer la distance restante sans tenir compte des coûts déjà accumulés.
Il est rapide mais ne garantit pas d'obtenir le chemin optimal.

**Paramètres :**
  - Fonction heuristique (h) : La fonction qui estime le coût restant pour atteindre l'objectif.
    Exemple : Distance Euclidienne ou Manhattan.
  - Règles de sélection des voisins : Choisit toujours le voisin avec la valeur heuristique la plus faible.

**Analyse :**
  - GBFS est très rapide mais ne garantit pas de solution optimale.
  - Il peut conduire à une solution incorrecte si l'heuristique est mal choisie.
  - Comparé à A*, il est plus rapide mais moins fiable car il ne considère pas les coûts accumulés.
  - Cet algorithme est utilisé dans des scénarios où la rapidité prime sur l'optimalité.

## Partie 4 : Navigation Autonome d’une Carte Connue avec TurtleBot<br>



## Partie 5 : Implémentation d’une tâche de Motion Planning (Planification de Mouvement)<br>



