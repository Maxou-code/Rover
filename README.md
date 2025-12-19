# 🤖 Rover – Robot ESP32 & Arduino Mega

## 📌 Présentation
**Rover** est un robot mobile basé sur un **ESP32** et un **Arduino Mega**, conçu pour la robotique expérimentale et éducative.  
Il combine la gestion des capteurs, la géolocalisation GPS et le contrôle des moteurs avec une **communication Wi-Fi** permettant un pilotage et un retour d’informations à distance.

---

## 🧠 Architecture générale
Le robot est composé de deux microcontrôleurs communiquant via une **liaison série (UART)** :

### 🔹 Arduino Mega
- Gestion des capteurs
- Lecture du module GPS
- Envoi des données à l’ESP32 via le port série

### 🔹 ESP32
- Contrôle des moteurs
- Contrôle des servomoteurs
- Communication Wi-Fi
- Réception et traitement des données provenant de l’Arduino Mega

---

## 🔌 Communication
- **ESP32 ↔ Arduino Mega** : communication série (UART)
- **Robot ↔ Client** : Wi-Fi
- **GPS** : connecté au port série de l’Arduino Mega

---

## 🔧 Capteurs utilisés (Arduino Mega)
- 📏 Capteur **ultrason** (distance)
- 🌡️ Capteur de **température**
- 💧 Capteur d’**humidité**
- 💡 Capteur de **luminosité**
- 🛰️ **Module GPS** (position, latitude, longitude)

---

## ⚙️ Actionneurs
- 🚗 Moteurs pilotés via une **carte de contrôle moteur** (ESP32)
- 🎯 **Servomoteur** contrôlé par l’ESP32

---

## 🌐 Fonctionnalités
- ✅ Communication série ESP32 ↔ Arduino Mega
- ✅ Lecture et transmission des données capteurs
- ✅ Géolocalisation GPS
- ✅ Contrôle des moteurs
- ✅ Contrôle des servomoteurs
- ✅ Communication Wi-Fi avec le robot
- 🔄 Extension possible (autonomie, interface web, IA, etc.)

---
