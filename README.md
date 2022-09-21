# Railcom-detector-on-ESP32

DETECTION DE RAILCOM canal1 (adresse DCC)

Ce programme fonctionne sur ESP32 et permet de lire l’adresse d’une locomotive équipée en Railcom.

La sortie TX du détecteur est reliée à l’entrée 14 de l’ESP32
L’entrée BRAKE de la carte moteur (LMD18200) est reliée à l’entrée 13 de l’ESP32

Bien sûr, il est nécessaire de disposer d’une centrale qui génère le CUTOUT pour le fonctionnement de Railcom comme celle présentée ici : https://github.com/BOBILLEChristophe/DCCxx-ESP32-Railcom

Mais toute autre centrale qui génère du DCC avec CUTOUT est compatible.

Cette station est présentée sur le forum LOCODUINO : https://forum.locoduino.org/index.php?topic=1352.0

Les fichiers Gerber du détecteur Railcom ainsi que la liste des composants seront bientôt disponibles.
