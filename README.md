# 🤖 Rover – Robot ESP32 & Arduino Mega

## 📌 Présentation

**Rover** est un robot mobile basé sur un **ESP32** et un **Arduino Mega**, conçu pour la robotique expérimentale et éducative.
Il combine la gestion des capteurs, la géolocalisation GPS et le contrôle des moteurs avec une **communication Wi-Fi**, permettant un pilotage et un retour d’informations à distance.
Une **Arduino Uno** peut être intégrée côté client pour faciliter le pilotage via un **joystick et un bouton** sur l’interface web.

---

## 🧠 Architecture générale

Le robot est composé de deux microcontrôleurs communiquant via une **liaison série (UART)** :

### 🔹 Arduino Mega

* Gestion des capteurs
* Lecture du module GPS
* Pilotage des **servomoteurs**
* Mesure de la tension de la batterie via un **pont diviseur de tension**
* Envoi des données à l’ESP32 via le port série
* Contrôle la partie logique des moteurs

### 🔹 ESP32

* Contrôle le **PWM** des moteurs
* Communication Wi-Fi
* Réception et traitement des données provenant de l’Arduino Mega
* Stream en temps réel via une caméra possédant 2 degrés de liberté

---

## 🔌 Communication

* **ESP32 ↔ Arduino Mega** : communication série (UART)
* **Robot ↔ Client** : Wi-Fi
* **GPS** : connecté au port série de l’Arduino Mega

---

## 🔧 Capteurs utilisés (Arduino Mega)

* 📏 Capteur **ultrason** (distance)
* 🌡️ Capteur de **température**
* 💧 Capteur d’**humidité**
* 💡 Capteur de **luminosité**
* 🛰️ **Module GPS** (position, latitude, longitude)
* 🔋 Tension de la batterie via **pont diviseur**

---

## ⚙️ Actionneurs

* 🚗 **Moteurs** : pilotés par l’ESP32 pour les PWM, le reste par l’Arduino Mega
* 🎯 **Servomoteurs** : pilotés par l’Arduino Mega

---

## 🌐 Fonctionnalités

* ✅ Communication série ESP32 ↔ Arduino Mega
* ✅ Lecture et transmission des données capteurs
* ✅ Géolocalisation GPS
* ✅ Contrôle des moteurs
* ✅ Contrôle des servomoteurs
* ✅ Communication Wi-Fi avec le robot
* ✅ Pilotage simplifié via une **Arduino Uno côté client** (Joystick + bouton)
* 🔄 Extension possible (autonomie, interface web, IA, etc.)

---

## ⚠️ Limitations

* Autonomie limitée
* Détection imprécise à courte distance
* GPS peu précis en intérieur

---

## 🧩 Modèles 3D

Les modèles 3D utilisés pour le robot **Rover** sont disponibles dans les **Releases GitHub**.

📦 Ils se trouvent dans la section **`Solid`** de chaque release et incluent notamment :

* Boîtier ESP32
* Boîtier pour capteur ultrason
* Support moteur
* Support pour planche à pain

Les fichiers sont fournis prêts à être **imprimés en 3D** et peuvent évoluer selon les versions du projet.
