# 🔌 Branchement

## ESP32

| Port              |  Pin ESP32 | Pin ESP32   | Port     |
|-------------------|------------|-------------|----------|
| Moteur 5V         | 5V         | 3.3V        | X        |
| Moteur GND        | GND        | IO16        | X        |
| Moteur IN4        | IO12       | IO0         | X        |
| Moteur IN3        | IO13       | GND         | GND Mega |
| Moteur IN2        | IO15       | 3.3V / 5V   | X        |
| Moteur IN1        | IO14       | RX          | TX Mega  |
| Servo moteur      | IO2        | TX          | RX Mega  |
| X                 | IO4        | GND         | X        |

## Capteur ultrason

| Capteur Ultrason Avant D | Port     | Capteur Ultrason Avant G | Port     | Capteur Ultrason Arrière | Port     |
| ------------------------ | -------- | ------------------------ | -------- | ------------------------ | -------- |
| 5V                       | 5V Mega  | 5V                       | 5V Mega  | 5V                       | 5V Mega  |
| Trig                     | 22 Mega  | Trig                     | 46 Mega  | Trig                     | 28 Mega  |
| Echo                     | 24 Mega  | Echo                     | 48 Mega  | Echo                     | 30 Mega  |
| GND                      | GND Mega | GND                      | GND Mega | GND                      | GND Mega |

| Capteur Ultrason Droite | Port     | Capteur Ultrason Gauche | Port     |
| ----------------------- | -------- | ----------------------- | -------- |
| 5V                      | 5V Mega  | 5V                      | 5V Mega  |
| Trig                    | 34 Mega  | Trig                    | 40 Mega  |
| Echo                    | 36 Mega  | Echo                    | 42 Mega  |
| GND                     | GND Mega | GND                     | GND Mega |

## Photorésistance

| Phtorésistance 1 | Port     | Phtorésistance 2 | Port     | Phtorésistance 3 | Port     | Phtorésistance 4 | Port     |
| ---------------- | -------- | ---------------- | -------- | ---------------- | -------- | ---------------- | -------- |
| 5V               | 5V Mega  | 5V               | 5V Mega  | 5V               | 5V Mega  | 5V               | 5V Mega  |
| OUT              | A0 Mega  | OUT              | A1 Mega  | OUT              | A2 Mega  | OUT              | A3 Mega  |
| GND              | GND Mega | GND              | GND Mega | GND              | GND Mega | GND              | GND Mega |

## Autre

| DHT | Port     | LED | Port     | GPS  | Port     | Servo moteur | Port      |
| --- | -------- | --- | -------- | ---- | -------- | ------------ | --------- |
| 5V  | 5V Mega  | IN  | 4        | 3,7V | EXT      | 5V           | 5V Mega   |
| OUT | 2 Mega   | GND | GND Mega | RX   | TX1 Mega | IN           | IO2 ESP32 |
| GND | GND Mega |     |          | TX   | RX1 Mega | GND          | GND Mega  |
|     |          |     |          | GND  | EXT      |              |           |
