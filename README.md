# Système de Retour Haptique avec Vision par Ordinateur

## Description du Projet

Projet réalisé dans le cadre du Master Systems for Rehabilitation (MSR) - Sorbonne Université.

Ce système d'assistance innovant a été conçu pour aider les personnes non-voyantes dans leur déplacement quotidien. Il combine la vision par ordinateur et le retour haptique pour détecter les obstacles et communiquer leur position à l'utilisateur via des vibrations. Développé initialement comme un système d'évitement d'obstacles, le projet a le potentiel d'évoluer vers une solution complète de navigation assistée.
Le système utilise une caméra de profondeur OAK-D pour détecter les obstacles et transmet l'information via des moteurs vibrants positionnés de manière ergonomique, permettant à l'utilisateur de percevoir intuitivement la direction des obstacles et de choisir un chemin sûr.
Ce projet s'inscrit dans une démarche d'innovation technologique au service de l'autonomie des personnes en situation de handicap visuel.

![image](https://github.com/user-attachments/assets/4a341eb2-9c87-4871-bae4-6df00adad622)

![image](https://github.com/user-attachments/assets/3d561fe4-55d5-4577-99b6-d30c16daa85f)
![image](https://github.com/user-attachments/assets/2ae4a0a7-4f23-4424-a05e-9986c4e1fdb9)


## Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Installation](#installation)
3. [Configuration de l'Espace de Travail](#configuration-de-lespace-de-travail)
4. [Nœuds ROS2](#nœuds-ros2)
5. [Partie Électronique](#partie-électronique)
6. [Partie Mécanique](#partie-mécanique)
7. [Documentation Utilisateur](#documentation-utilisateur)

## Vue d'ensemble

Ce projet implémente un système de détection d'obstacles et de retour haptique guidé par vision par ordinateur. Il utilise ROS2, une caméra de profondeur OAK-D, et des moteurs vibrants LRA (Linear Resonant Actuator) contrôlés par une carte Arduino MKR WiFi 1010.

## Installation

### Prérequis Système
- Ubuntu 22.04 LTS
- Python 3.10 ou supérieur
- Git
- VS Code avec PlatformIO

### ROS2 Humble
```bash
wget -c https://raw.githubusercontent.com/ros/rosdistro/master/ros.key && sudo apt-key add ros.key && sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list' && sudo apt update && sudo apt install ros-humble-desktop-full
```

### Configuration Caméra OAK-D

1. Documentation officielle :
   - [Documentation Luxonis OAK-D](https://docs.luxonis.com/projects/api/en/latest/)
   - [Exemples DepthAI](https://github.com/luxonis/depthai-python/tree/main/examples)

2. Installation des dépendances :
   ```bash
   # Installation des dépendances système DepthAI
   sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash

   # Installation de la bibliothèque Python DepthAI
   python3 -m pip install depthai

   # Installation des dépendances ROS2
   sudo apt install ros-humble-depthai-ros
   ```

3. Test de l'installation :
   ```bash
   # Cloner les exemples
   git clone https://github.com/luxonis/depthai-python.git
   cd depthai-python/examples
   
   # Installer les dépendances Python
   python3 install_requirements.py
   
   # Tester la caméra
   python3 ColorCamera/rgb_preview.py
   ```

4. Vérification des règles udev :
   ```bash
   # Vérifier que la caméra est détectée
   lsusb | grep MyriadX
   
   # Si nécessaire, recharger les règles udev
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### Configuration Arduino et PlatformIO (À lancer avant ROS2)

1. Installation de l'environnement de développement :
   ```bash
   # Installation de VS Code
   sudo apt update
   sudo apt install software-properties-common apt-transport-https wget
   wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
   sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
   sudo apt update
   sudo apt install code
   ```

2. Configuration de PlatformIO :
   - Ouvrir VS Code
   - Aller dans Extensions (Ctrl+Shift+X)
   - Rechercher "PlatformIO IDE"
   - Installer l'extension
   - Redémarrer VS Code

3. Structure du projet Arduino :
   ```
   haptic-arduino/
   ├── platformio.ini
   ├── src/
   │   └── main.cpp              # Code principal Arduino
   ├── lib/
   │   ├── Haptic_DRV2605/      # Bibliothèque des moteurs
   │   └── README.md
   └── include/
       └── README.md
   ```

4. Configuration détaillée dans `platformio.ini` :
   ```ini
   [env:mkrwifi1010]
   platform = atmelsam
   board = mkrwifi1010
   framework = arduino
   
   ; Configuration moniteur série
   monitor_speed = 115200
   monitor_flags = 
       --parity
       N
       --encoding
       UTF-8
   
   ; Dépendances externes
   lib_deps =
       arduino-libraries/WiFiNINA @ ^1.8.14
       bblanchon/ArduinoJson @ ^6.21.3
       Wire
       adafruit/Adafruit DRV2605 Library @ ^1.1.2
   
   ; Options de build
   build_flags =
       -D DEBUG_MODE
       -D WIFI_SSID=\"HapticArduino\"
       -D WIFI_PASS=\"haptic123\"
   
   ; Configuration mémoire
   board_build.f_cpu = 48000000L
   upload_speed = 115200
   ```

5. Compilation et téléversement :
   ```bash
   # Dans le dossier du projet Arduino
   pio run -t upload
   ```

6. Vérification du fonctionnement :
   ```bash
   # Ouvrir le moniteur série
   pio device monitor
   ```
   - Vérifier que l'Arduino crée bien le point d'accès WiFi
   - Confirmer l'initialisation des moteurs
   - Noter l'adresse IP (par défaut 192.168.4.1)

## Nœuds ROS2

### Objectifs des Nœuds

#### 1. Nœud Processeur de Profondeur (processor_node.py)
Ce nœud reçoit les données brutes de la caméra de profondeur et les traite pour identifier les obstacles.
Il normalise les données de profondeur et publie une image traitée où les obstacles proches sont plus sombres.

#### 2. Nœud de Projection 2D (2d.py)
Ce nœud transforme l'image de profondeur traitée en une projection 2D avec détection d'obstacles.
Il analyse la position des obstacles et détermine de quel côté l'utilisateur doit se diriger.

#### 3. Nœud de Publication Haptique (haptic_publisher.py)
Ce nœud gère l'activation des différents moteurs haptiques selon les commandes reçues.
Il maintient deux groupes de moteurs (gauche et droite) et coordonne leurs activations.

#### 4. Nœud Bridge WiFi (wifi_bridge.py)
Ce nœud établit la communication WiFi entre ROS2 et l'Arduino.
Il convertit les commandes ROS2 en messages JSON et les envoie à l'Arduino via UDP.

### Lancement des Nœuds

#### 1. Lancement de la Caméra
```bash
# Lance la caméra OAK-D et publie les données de profondeur
colcon build --symlink-install --packages-select depthai_bridge
colcon build --symlink-install --packages-select depthai_examples
source install/setup.bash
ros2 launch depthai_examples stereo_inertial_node.launch.py depth:=true
```

#### 2. Lancement des Nœuds de Traitement
```bash
# Compile et lance les nœuds de traitement de profondeur
colcon build --packages-select depth_processor
source install/setup.bash
ros2 run depth_processor processor_node
ros2 run depth_processor 2d_node
```

#### 3. Lancement des Nœuds Haptiques
```bash
# Compile et lance les nœuds de contrôle haptique
colcon build --packages-select haptic_control_interfaces haptic_control_pkg
source install/setup.bash
ros2 run haptic_control_pkg haptic_publisher
ros2 run haptic_control_pkg haptic_bridge
```

### Tests Rapides
```bash
# Active les moteurs gauches
ros2 topic pub /haptic_side haptic_control_interfaces/msg/SideCommand "{side: 2}"

# Active les moteurs droits
ros2 topic pub /haptic_side haptic_control_interfaces/msg/SideCommand "{side: 1}"
```

## Partie Électronique

### Liste du Matériel
- 1x Arduino MKR WiFi 1010
- 8x Moteurs LRA
- 1x Multiplexeur I2C TCA9548A
- 8x Drivers DRV2605
- 1x PCB personnalisée
- Composants:
  - Régulateur 3.3V
  - Condensateurs de découplage
  - Résistances pull-up
  - Connecteurs moteurs

### Carte PCB

La carte PCB a été conçue pour gérer 6 moteurs haptiques de type LRA. Elle intègre les composants suivants :

#### Composants Principaux
- 1x Arduino MKR WiFi 1010 (microcontrôleur principal)
- 6x Drivers DRV2605 (contrôle des moteurs LRA)
- 1x Multiplexeur I2C TCA9548A (gestion des multiples drivers)
- 6x Connecteurs JST femelles (connexion des moteurs)

#### Connexions
- Chaque moteur se connecte via un connecteur JST 2 pins pour :
  - VCC (alimentation)
  - GND (masse)

#### Communication
- Bus I2C distribué via le multiplexeur
- Le multiplexeur permet de gérer les 6 drivers avec une seule paire de lignes I2C

![image](https://github.com/user-attachments/assets/6659c939-70c5-4f29-9acb-38315914d734)
![image](https://github.com/user-attachments/assets/ee2448f7-97e9-4d0e-83f1-e2261169bead)

*Note : Tous les fichiers liés à la PCB sont disponibles dans le dossier `PCB_designing/` du projet*


### Configuration WiFi Arduino
- Mode: Point d'accès (AP)
- SSID: "HapticArduino"
- Password: "haptic123"
- IP Arduino: 192.168.4.1
- Port UDP: 8888
- Format: JSON

### Format des Messages JSON
```json
{
    "motor": 0-7,           // ID du moteur
    "waveform": 0-123,      // Pattern de vibration
    "activate": true/false,  // Activation/désactivation
    "intensity": 0.0-1.0    // Intensité de vibration
}
```

## Partie Mécanique

Le système comporte plusieurs pièces imprimées en 3D, chacune ayant une fonction spécifique :

### 1. Support Principal
- Pièce servant de base au système
- Conçue avec un système de clips pour maintenir la PCB
- Intègre un couvercle arrière pour protéger l'électronique
- Dimensions optimisées pour s'adapter à la PCB et au boîtier de la caméra

![image](https://github.com/user-attachments/assets/651263d2-5161-43fb-8e46-f35fd7bd948e)

### 2. Support Moteur et Driver (Prototype)
- Prototype initial permettant d'accueillir :
  - Le moteur LRA
  - Son driver DRV2605
  - Les connexions nécessaires
- Bien que non utilisé dans la version finale, ce design reste pertinent pour des tests unitaires ou des prototypes

![image](https://github.com/user-attachments/assets/2c4c2f67-6ed1-4ed1-9ad1-a90cf950a989)
![image](https://github.com/user-attachments/assets/f685b6a9-2f8d-4834-818a-eff6cb87fe1f)


### 3. Support Moteur Simple
- Version optimisée pour maintenir uniquement le moteur LRA
- Design minimaliste pour réduire l'encombrement
- Système de fixation sécurisé pour le moteur
- Adapté pour l'intégration dans le système final

![image](https://github.com/user-attachments/assets/c0f505d1-bf51-4839-88c3-2889e982cc81)
![image](https://github.com/user-attachments/assets/fef0ad0b-dc84-4f96-9e60-2b317578e54c)


*Note : Tous les fichiers STL et les sources CAO sont disponibles dans le dossier `Solidworks/` du projet*

## Documentation Utilisateur

[Manuel Utilisateur PDF](lien_vers_manuel.pdf)

## Résolution des Problèmes

### Caméra non détectée
- Vérifier connexion USB
- Vérifier règles udev
- Redémarrer le service udev

### Erreurs WiFi
- Vérifier que l'Arduino crée bien le point d'accès
- Vérifier dans le moniteur série (115200 baud) le statut de la connexion
- Confirmer l'IP et le port dans le code

### Moteurs non réactifs
- Vérifier l'adresse du multiplexeur I2C (0x70)
- Vérifier les connexions des moteurs
- Utiliser le moniteur série pour voir les messages d'initialisation

## Contact
Pour toute question sur ce projet, contactez les auteurs via LinkedIn.

## Auteurs et Remerciements

### Auteurs
- Amal BECHEKER
- Salah Eddine HAMIZI
- Vincent FONROUGE

### Remerciements
Nous tenons à remercier particulièrement :
- M. Fabien VÉRITÉ pour la gestion et la coordination du Master MSR
- L'équipe pédagogique de Sorbonne Université

Projet réalisé dans le cadre du Master MSR - Sorbonne Université, 2024.
