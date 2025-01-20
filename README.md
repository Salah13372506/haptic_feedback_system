# Système de Retour Haptique avec Vision par Ordinateur

Ce projet implémente un système de détection d'obstacles et de retour haptique guidé par vision par ordinateur. Il utilise ROS2, une caméra de profondeur OAK-D, et des moteurs vibrants LRA (Linear Resonant Actuator) contrôlés par une carte Arduino MKR WiFi 1010.

## Table des Matières

1. [Installation](#installation)
2. [Configuration de l'Espace de Travail](#configuration-de-lespace-de-travail)
3. [Compilation et Exécution](#compilation-et-exécution)
4. [Partie Électronique](#partie-électronique)
5. [Partie Mécanique](#partie-mécanique)
6. [Documentation Utilisateur](#documentation-utilisateur)

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

### Dépendances Camera
```bash
# Installation des dépendances DepthAI
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash

# Installation de la bibliothèque Python DepthAI
python3 -m pip install depthai

# Test de l'installation
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 install_requirements.py
python3 ColorCamera/rgb_preview.py
```

### Configuration PlatformIO
1. Installer VS Code depuis [Visual Studio Code](https://code.visualstudio.com/)
2. Installer l'extension PlatformIO IDE dans VS Code
3. Ouvrir le projet Arduino:
```bash
git clone https://github.com/votre-repo/haptic-arduino.git
```

4. Configuration dans `platformio.ini`:
```ini
[env:mkrwifi1010]
platform = atmelsam
board = mkrwifi1010
framework = arduino
lib_deps =
    arduino-libraries/WiFiNINA
    bblanchon/ArduinoJson
    Wire
    adafruit/Adafruit DRV2605 Library
monitor_speed = 115200
```

5. Structure du projet Arduino:
```
haptic-arduino/
├── src/
│   └── main.cpp                 # Code principal
├── lib/
│   └── Haptic_DRV2605/         # Bibliothèque personnalisée pour les moteurs
└── platformio.ini
```

6. Configuration WiFi dans le code:
```cpp
const char* ap_ssid = "HapticArduino";     // Nom du réseau
const char* ap_password = "haptic123";      // Mot de passe (minimum 8 caractères)
const int localPort = 8888;                 // Port UDP
IPAddress local_ip(192, 168, 4, 1);        // IP fixe
```

## Configuration de l'Espace de Travail

```bash
# Créer l'espace de travail
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/votre-repo/haptic-feedback-system.git
cd ..

# Installer les dépendances
rosdep install --from-paths src --ignore-src -r -y
```

## Compilation et Exécution

### Configuration Camera
```bash
# Compilation des packages caméra
colcon build --symlink-install --packages-select depthai_bridge
colcon build --symlink-install --packages-select depthai_examples
source install/setup.bash

# Lancement de la caméra
ros2 launch depthai_examples stereo_inertial_node.launch.py depth:=true
```

Note: Dans RViz2, ajoutez le topic `/stereo/converted_depth`

### Nœuds de Traitement
```bash
colcon build --packages-select depth_processor
source install/setup.bash
ros2 run depth_processor processor_node
ros2 run depth_processor 2d_node
```

### Nœuds Haptiques
```bash
colcon build --packages-select haptic_control_interfaces haptic_control_pkg
source install/setup.bash
ros2 run haptic_control_pkg haptic_publisher
ros2 run haptic_control_pkg haptic_bridge

# Tests:
ros2 topic pub /haptic_side haptic_control_interfaces/msg/SideCommand "{side: 2}"   # gauche
ros2 topic pub /haptic_side haptic_control_interfaces/msg/SideCommand "{side: 1}"   # droite
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

### Configuration Moteurs
- Type: Linear Resonant Actuator (LRA)
- Mode: REGISTER_MODE
- Multiplexeur I2C pour gérer 8 moteurs
- Adresse TCA9548A: 0x70

## Partie Mécanique

- [Fichiers CAO](lien_a_venir)
- [Fichiers STL](lien_a_venir)
- [Instructions d'assemblage](lien_a_venir)

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

## License
[À définir]

## Contact
Pour toute question, ouvrez une issue sur ce dépôt.
