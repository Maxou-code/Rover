# 🔧 Améliorations matérielles prévues pour le robot

Ce document présente les principales améliorations matérielles nécessaires afin d’augmenter la **fiabilité**, les **performances** et l’**autonomie** du robot.

---

## 🧱 1. Remplacement du châssis

### Problème actuel
- Châssis en **carton** :
  - Faible rigidité
  - Sensible à l’humidité
  - Mauvaise tenue mécanique des moteurs et capteurs

### Amélioration proposée
- Remplacer par un châssis en **acrylique (PMMA)** :
  - Plus rigide et durable
  - Meilleure précision mécanique
  - Support propre pour moteurs et électronique

---

## 🌤️ 2. Remplacement des photorésistances par des modules autonomes

### Problème actuel
- Photorésistances montées sur **planche à pain**
- Résistances séparées
- Câblage fragile et encombrant
- Risque de faux contacts et de mesures instables

### Amélioration proposée
- Utiliser des **modules photorésistance intégrés** :
  - Photorésistance + résistance déjà intégrée
  - Sortie analogique prête à l’emploi
  - Connecteurs simples (VCC / GND / OUT)

### Bénéfices
- Câblage plus propre et fiable
- Gain de place
- Mesures plus stables
- Intégration facilitée sur un shield custom

---

## 🧩 3. Boîtiers 3D pour les composants

### Objectifs
- Protéger l’électronique
- Améliorer l’organisation interne
- Faciliter la maintenance

### Actions
- Impression 3D de boîtiers dédiés pour :
  - ESP32
  - Arduino Mega
  - Driver moteurs
  - Batterie
  - Capteurs

### Bénéfices
- Réduction des courts-circuits
- Meilleure dissipation thermique
- Aspect professionnel

---

## 🔌 4. Shield custom pour l’Arduino Mega

### Problème actuel
- Câblage complexe
- Risque de faux contacts
- Difficulté de debug

### Amélioration proposée
- Création d’un **shield personnalisé** pour :
  - Connecter les capteurs
  - Centraliser les alimentations
  - Simplifier le câblage

### Avantages
- Montage plus propre
- Maintenance facilitée
- Évolutivité du projet

---

## ⚡ 5. Remplacement du driver moteur L298N (priorité critique)

### Limites du L298N
- Très mauvais rendement (< 50%)
- Forte chute de tension (2–4 V)
- Chauffe importante
- Consommation excessive de la batterie

### Solution recommandée
- Remplacer par un **driver moteur à MOSFET** :
  - BTS7960 (recommandé)
  - VNH5019
  - DRV8871

### Bénéfices
- Rendement nettement supérieur
- Moins de pertes énergétiques
- Autonomie augmentée (jusqu’à ×2)
- Meilleure gestion du couple moteur

---

## 🔋 6. Améliorations électriques complémentaires

- Alimentation de l’ESP32 via un **buck converter** (LM2596 / MP1584)
- Condensateurs antiparasites sur les moteurs
- Filtrage de l’alimentation (condensateurs de découplage)
- Batterie plus adaptée (cellules Li-ion high-drain ou configuration 3S2P)

---

## ✅ Conclusion

Ces améliorations permettront :
- Une **meilleure autonomie**
- Une **fiabilité accrue**
- Un robot plus **robuste et professionnel**
- Une base saine pour des évolutions futures

Le remplacement du **L298N** et l’amélioration de l’architecture électrique sont les **priorités absolues**.
