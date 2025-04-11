#!/usr/bin/env python

import rospy
import subprocess
import signal
import time
import os

def launch_process(command):
    """Lance un processus shell dans un terminal séparé et retourne le process"""
    return subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

def main():
    rospy.init_node('noeud2', anonymous=True)

    rospy.loginfo("===== Lancement SLAM Mapping avec TurtleBot3 =====")

    control_mode = rospy.get_param("~control_mode", "keyboard")  # "keyboard" ou "joystick"

    # 1. Lancer Gazebo avec TurtleBot3
    rospy.loginfo("[1] Lancement de Gazebo...")
    gazebo = launch_process("xterm -hold -e roslaunch turtlebot3_gazebo turtlebot3_world.launch")
    time.sleep(5)

    # 2. Lancer le SLAM (gmapping)
    rospy.loginfo("[2] Lancement de SLAM avec gmapping...")
    slam = launch_process("xterm -hold -e roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")
    time.sleep(3)

    # 3. Lancer le contrôle
    rospy.loginfo("[3] Lancement du contrôle : {}".format(control_mode))
    if control_mode == "keyboard":
        teleop = launch_process("roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch")
    elif control_mode == "joystick":
        teleop = launch_process("roslaunch teleop_twist_joy teleop.launch")
    else:
        rospy.logerr("Mode de contrôle non reconnu. Utilisez 'keyboard' ou 'joystick'.")
        return

    rospy.loginfo("Déplacement en cours... Utilise le contrôle pour explorer la carte.")
    rospy.loginfo("Quand tu veux arrêter : Ctrl+C => la carte sera sauvegardée.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Arrêt détecté, sauvegarde de la carte...")

        # 4. Sauvegarder la carte
        subprocess.call("rosrun map_server map_saver -f ~/map", shell=True)
        rospy.loginfo("Carte sauvegardée sous ~/map.pgm et ~/map.yaml")

        # Terminer les processus
        os.killpg(os.getpgid(gazebo.pid), signal.SIGTERM)
        os.killpg(os.getpgid(slam.pid), signal.SIGTERM)
        os.killpg(os.getpgid(teleop.pid), signal.SIGTERM)
        rospy.loginfo("Tous les processus terminés.")

if __name__ == '__main__':
    main()
