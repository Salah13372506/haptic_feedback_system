#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Haptic_DRV2605.h"

// Configuration WiFi
const char* ap_ssid = "HapticArduino";     // Network Name
const char* ap_password = "haptic123";      // Password (8 characters minimum)
const int localPort = 8888;


// Fixed IP config
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Configuration TCA9548A
#define TCA9548A_ADDRESS 0x70

// LRA Motors Array
Haptic_DRV2605 haptic[8];

// UDP Data buffer
WiFiUDP udp;
char packetBuffer[256];

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setupWiFi() {
    Serial.println("Configuration du point d'accès WiFi...");
    
    // Créer le réseau WiFi
    if (WiFi.beginAP(ap_ssid, ap_password) != WL_AP_LISTENING) {
        Serial.println("Erreur création point d'accès !");
        while (true); // Arrêt si échec de création
    }

    // Laisser le temps au point d'accès de démarrer
    delay(2000);

    // Vérifier le statut
    while (WiFi.status() != WL_AP_LISTENING) {
        Serial.println("En attente du démarrage du point d'accès...");
        delay(500);
    }
    
    // Afficher les informations de connexion
    Serial.println("Point d'accès créé avec succès !");
    Serial.println("Configuration réseau :");
    Serial.print("    SSID : ");
    Serial.println(ap_ssid);
    Serial.print("    IP : ");
    Serial.println(WiFi.localIP());
    Serial.print("    Port : ");
    Serial.println(localPort);
    
    // Démarrer le serveur UDP sur le port spécifié
    udp.begin(localPort);
    Serial.println("Serveur UDP démarré");
    
    // Afficher un message pour indiquer que l'Arduino est prêt
    Serial.println("Arduino prêt à recevoir des commandes UDP");
}


void setupHapticMotors() {
  Wire.begin();
  
  // Initialisation de tous les moteurs
  for (uint8_t channel = 0; channel < 8; channel++) {
    tcaSelect(channel);
    
    if (haptic[channel].begin() == HAPTIC_SUCCESS) {
      Serial.print("Moteur initialisé sur le canal ");
      Serial.println(channel);
      
      haptic[channel].setActuatorType(LRA);
      haptic[channel].setMode(REGISTER_MODE);
    } else {
      Serial.print("Erreur d'initialisation moteur canal ");
      Serial.println(channel);
    }
  }
}

void handleHapticCommand(int motor_id, int waveform, bool activate, float intensity) {
  if (motor_id < 0 || motor_id > 7) return;
  
  tcaSelect(motor_id);
  
  if (activate) {
    // Convertir l'intensité (0.0-1.0) en valeur compatible DRV2605 
    uint8_t strength = intensity * 127;
    haptic[motor_id].setRealtimeValue(strength);
    
    haptic[motor_id].setWaveform(0, waveform);
    haptic[motor_id].setWaveform(1, 0);  // Stop
    haptic[motor_id].go();  // Démarrer la séquence
  } else {
    haptic[motor_id].stop();
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  setupWiFi();
  setupHapticMotors();
}

void loop() {
    // Vérifier si des données UDP sont disponibles
    int packetSize = udp.parsePacket();
    if (packetSize) {
        Serial.print("Paquet UDP reçu, taille: ");
        Serial.println(packetSize);
        
        // Lire le paquet
        int len = udp.read(packetBuffer, 255);
        if (len > 0) {
            packetBuffer[len] = 0;  // Null-terminate string
            Serial.print("Message reçu: ");
            Serial.println(packetBuffer);
            
            // Parse le JSON
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, packetBuffer);
            
            if (!error) {
                // Extraire les valeurs
                int motor = doc["motor"];
                int waveform = doc["waveform"];
                bool activate = doc["activate"];
                float intensity = doc["intensity"];
                
                Serial.println("Commande décodée:");
                Serial.print("  Motor: "); Serial.println(motor);
                Serial.print("  Waveform: "); Serial.println(waveform);
                Serial.print("  Activate: "); Serial.println(activate);
                Serial.print("  Intensity: "); Serial.println(intensity);
                
                // Appliquer la commande
                handleHapticCommand(motor, waveform, activate, intensity);
            } else {
                Serial.print("Erreur parsing JSON: ");
                Serial.println(error.c_str());
            }
        }
    }
}
