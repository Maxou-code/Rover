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

| Capteur Ultrason Avant D | Port     || Capteur Ultrason Avant G | Port     || Capteur Ultrason Arrière | Port     || Capteur Ultrason Droite | Port     || Capteur Ultrason Gauche | Port     |
| ------------------------ | -------- || ------------------------ | -------- || ------------------------ | -------- || ----------------------- | -------- || ----------------------- | -------- |
|                          |          ||                          |          ||                          |          ||                         |          ||                         |          |
| 5V                       | 5V Mega  || 5V                       | 5V Mega  || 5V                       | 5V Mega  || 5V                      | 5V Mega  || 5V                      | 5V Mega  |
| Trig                     | 22 Mega  || Trig                     | 46 Mega  || Trig                     | 28 Mega  || Trig                    | 34 Mega  || Trig                    | 40 Mega  |
| Echo                     | 24 Mega  || Echo                     | 48 Mega  || Echo                     | 30 Mega  || Echo                    | 36 Mega  || Echo                    | 42 Mega  |
| GND                      | GND Mega || GND                      | GND Mega || GND                      | GND Mega || GND                     | GND Mega || GND                     | GND Mega |
