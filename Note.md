### NOTE PROJET ROVER

## Wi-Fi

# Résumé des améliorations clés

- Antenne directionnelle 8+ dBi pour plus de portée.
- Puissance TX max (19.5 dBm) sur l’ESP32.
- Canal Wi-Fi optimisé (1, 6 ou 11).
- ESP32 en hauteur et sans obstacles.
- Antenne puissante côté PC ou répéteur Wi-Fi.

---

### 📡 **1. Améliorer l’antenne**  
- **Utiliser une antenne directionnelle** (8+ dBi) pour concentrer le signal.  
- **Prendre une antenne avec un câble coaxial court** pour éviter les pertes de signal.  
- **Tester différentes orientations** pour optimiser le gain.  

---

### ⚡ **2. Augmenter la puissance d’émission de l’ESP32**  
- Régler la puissance TX au maximum **(19.5 dBm, soit ~100 mW)** :  
  ```cpp
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  ```
- Désactiver **Bluetooth** si non utilisé pour éviter les interférences en 2.4 GHz.  

---

### 📶 **3. Optimiser le canal Wi-Fi**  
- **Utiliser la bande 2.4 GHz** (plus grande portée que 5 GHz).  
- Choisir un canal **peu encombré** (1, 6 ou 11).  
- Scanner les réseaux environnants pour éviter les interférences :  
  ```cpp
  int numNetworks = WiFi.scanNetworks();
  ```

---

### 📍 **4. Améliorer la position de l’ESP32**  
- **Le placer en hauteur** (au moins 2-3m du sol).  
- **Éviter les obstacles** (murs, métal, objets réfléchissants).  
- **Orienter l’antenne correctement** (surtout pour une directionnelle).  

---

### 💻 **5. Améliorer la réception côté PC**  
- Utiliser une **antenne Wi-Fi USB puissante** (8+ dBi).  
- Installer un **répéteur Wi-Fi** entre l’ESP32 et le PC si besoin.  

---

### 🔄 **6. Utiliser un mode Wi-Fi optimisé**  
- Si l’ESP32 est un **point d’accès (AP)**, activer le mode **Long Range (LR)** :  
  ```cpp
  WiFi.softAP("ESP32_AP", "password", 1, 0, 4);
  ```
- Tester différents **protocoles Wi-Fi** (802.11b a une meilleure portée que 802.11n).  

---

### 🛰 **7. Si besoin d’encore plus de portée**  
- Ajouter **un ampli Wi-Fi** entre l’ESP32 et l’antenne.  
- Passer sur du **Wi-Fi longue portée** (ex: Ubiquiti).  
- Si **besoin de > 1 km**, envisager **LoRa** à la place du Wi-Fi.  

---

----------------------------------------------------------------------------------------------

---

### 🔹 **1. Antennes USB Wi-Fi directionnelles pour PC**  
Ces antennes sont souvent utilisées pour **capter des réseaux distants** et peuvent être une solution pour interagir avec l’ESP32 à plusieurs kilomètres.  

#### **📡 Types d’antennes USB Wi-Fi pour PC :**  
1️⃣ **Antenne Yagi directionnelle USB** (8-15 dBi) → portée jusqu’à **2-5 km**  
2️⃣ **Antenne panneau directionnelle USB** (12-18 dBi) → portée jusqu’à **5-10 km**  
3️⃣ **Parabole Wi-Fi USB (Grid, Satellite)** (19-30 dBi) → portée jusqu’à **15 km et plus**  

**Exemples :**  
- ✅ **Alfa AWUS036NH + Yagi 16 dBi** (~3-5 km)  
- ✅ **Alfa AWUS036ACH + Panneau 18 dBi** (~5-10 km)  
- ✅ **Parabole Wi-Fi USB (Grid 24 dBi)** (~15+ km)  

Ces antennes **se branchent directement en USB** sur le PC et offrent une **meilleure réception Wi-Fi** qu’une clé USB classique.  

---

### 🔹 **2. Est-ce que ça permet de mieux capter un ESP32 avec antenne externe ?**  
✅ **Oui !** Une antenne directionnelle USB sur le PC **peut capter plus loin** un ESP32 équipé d’une **antenne externe**.  

💡 **Ce qu’il faut pour une connexion longue distance entre ESP32 et PC :**  
- **ESP32 avec antenne directionnelle 8+ dBi** → meilleure émission.  
- **PC avec antenne USB directionnelle 16+ dBi** → meilleure réception.  
- **Ligne de vue dégagée** entre les deux (pas de murs, obstacles).  

Si l’ESP32 est configuré en **point d’accès (AP)**, le PC pourra s’y connecter en Wi-Fi directement, même à plusieurs kilomètres.  

---

### 🔹 **3. Astuces pour booster encore plus la connexion**  
🔸 **ESP32 en hauteur (2-3m minimum)** pour réduire les interférences.  
🔸 **Utiliser un canal Wi-Fi peu encombré (1, 6, 11 sur 2.4 GHz)**.  
🔸 **Activer la puissance TX max sur l’ESP32** :  
```cpp
WiFi.setTxPower(WIFI_POWER_19_5dBm);
```  
🔸 **Utiliser un répéteur Wi-Fi** si la connexion est instable sur longue distance.  

---
