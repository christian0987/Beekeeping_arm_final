# Beekeeping_arm
# README



### Description

Projet de robotique visant à automatiser la manipulation de ruches à l’aide d’un bras robotique à 6 degrés de liberté.

Développé au sein du Laboratoire de Robotique Agricole de Qingdao City University, le projet permet au bras robotique de percevoir son environnement et de manipuler différents éléments d’une ruche.

Le système utilise une caméra pour détecter les couvercles et les cadres de cette ruche grâce à différentes techniques de traitement d’image. Les informations obtenues sont ensuite utilisées pour guider les mouvements du bras et réaliser différentes séquences de manipulation.

### Fonctionnalites principales

-Détection du cadre : Identification du rectangle externe (couvercle) par traitement d'image a l'aide de masques HSV et d'operations morphologiques

-Alignements iteratifs : Alignement automatique des axes J0 (vertical), J1 (angulaire) et J3 (centre horizontal) par asservissement visuel

-Detection des cadres : Identification des cadres par projection horizontale apres redressement de l'image par transformation de perspective

-Descente calculee : Descente du robot à X cm du cadre avec compensations cinematiques sur les axes J0 et J3

-Inspection des barres : Navigation vers chaque barre et inspection par rotation de l'axe J5 a +90° et -90°

-Retour home : Retour automatique de tous les joints a 0 avec verification des positions finales

-Interface par fichier de commandes : Permet de modifier la sequence sans recompiler ni modifier le programme principal

-Diagnostic integre : Fonction CHECK permettant d'afficher l'etat complet du systeme (positions, Z, etat de la descente)

### Pre-requis

#### Dependances Python

```
pip install opencv-python numpy rclpy
```

#### ROS2

Le programme est concu pour fonctionner avec ROS2 et necessite les topics suivants :

- Publishers :
  - /arm/joint_position (Float64MultiArray) - Position des articulations du bras
  - /wrist/joint_position (Float64MultiArray) - Position des articulations du poignet

- Subscribers :
  - /arm/joint_state (Float64MultiArray) - Etat du bras
  - /wrist/joint_state (Float64MultiArray) - Etat du poignet

#### Materiel

- Camera (testee avec index 2, modifiable dans le code)
- Robot avec 6 axes (J0 a J5)
- Couvercles et cadres de la ruche



### Interface Utilisateur

#### Commandes Clavier

| Touche | Action |
|--------|--------|
| a | Alignement vertical (J0) |
| g | Alignement angulaire (J1) |
| v | Alignement vertical centre (J3) |
| s | Sequence complete + inspection |
| b | Annuler le figage (ROI) |
| d | Activer/desactiver le mode debug |
| c | Afficher les coordonnees 3D |
| f | Charger un fichier de commandes |
| h | Afficher l'aide |
| q | Quitter |

#### Fichier de Commandes

Le programme accepte des fichiers de commandes pour automatiser les sequences. Cette approche permet de separer la definition de la tache robotique de son implementation logicielle.

##### Commandes disponibles

| Commande | Syntaxe | Description |
|----------|---------|-------------|
| V | V v0 v1 v2 v3 v4 v5 | Vitesses des joints J0..J5 |
| H | H 1 | Retour home (tout a 0) |
| I | I | Initialisation (attente positions ROS) |
| A | A | Alignement complet + mesure Z |
| D | D hauteur_cm | Descente a X cm |
| B | B 1,2,3,4 | Navigation vers les barres |
| W | W secondes | Attendre X secondes |
| P | P j0 j1 j2 j3 j4 j5 | Position absolue |
| M | M dj0 dj1 dj2 dj3 dj4 dj5 | Mouvement relatif |
| Z | Z | Mesurer la distance Z |
| R | R angle | Rotation J5 a X degres |
| C | C | Afficher les positions actuelles |
| CHECK | CHECK | Diagnostic complet |
| S | S fichier | Sauvegarder la configuration |
| L | L fichier | Charger une configuration |
| HELP | HELP | Afficher l'aide |
| QUIT | QUIT | Quitter |

##### Exemple de fichier de commandes

```
# Initialisation
I
W 1

# Alignement complet
A
W 2

# Verification des positions
C
W 1

# Descente a 2 cm
D 2
W 2

# Navigation vers les barres 1, 2 et 3
B 1,2,3
W 2

# Retour home
H 1

# Verification finale
C
```

### Sequence Complete

#### Phase 1 : Alignements

1. Alignement angulaire (J1) : Aligne le cadre horizontalement
2. Alignement vertical (J0) : Centre le cadre verticalement
3. Alignement vertical centre (J3) : Centre le cadre horizontalement
4. Mesure Z : Calcule la distance au cadre
5. Enregistrement J0 centre : Sauvegarde la position centrale

#### Phase 2 : Descente

1. Calcul des distances : Base sur la mesure Z
2. Descente J2 : 2/3 de la distance totale
3. Descente J4 : 1/3 de la distance totale
4. Compensations : J0 (horizontal) et J3 (vertical)

#### Phase 3 : Navigation + Inspection

Pour chaque barre demandee :

1. Deplacement J0 : Positionnement sur la barre
2. Remontee J2 : +200 mm
3. Rotation J5 a +90 degres : Inspection cote 1
4. Attente : 5 secondes
5. Rotation J5 a -90 degres : Inspection cote 2
6. Attente : 5 secondes
7. Retour J5 a 0 degres
8. Descente J2 : Retour a la position initiale

#### Phase 4 : Retour Home

1. Remise de tous les joints a 0 (J0 a J5)
2. Verification des positions finales

### Depannage

#### Problemes courants

**Aucune camera detectee**
```
# Verifier le peripherique
ls /dev/video*
# Modifier l'index dans le code si necessaire
self.cap = cv2.VideoCapture(2)  # Changer le 2
```

**Aucun cadre detecte**
- Verifier l'eclairage
- Ajuster les seuils de couleur dans les parametres
- Verifier que le cadre est bien visible

**Positions ROS non recues**
```
# Verifier les topics
ros2 topic list
# Verifier les messages
ros2 topic echo /arm/joint_state
```

**Z non mesure**
- Faire un alignement complet (touche a)
- Utiliser la commande Z dans un fichier

#### Messages d'erreur

| Message | Solution |
|---------|----------|
| Aucun cadre detecte | Verifier la camera et l'eclairage |
| Z non mesure ! | Faire un alignement (A) ou mesure Z |
| J2 non descendu ! | Verifier la descente automatique |
| Barre X hors limite | Verifier OFFSET_PINCE ou rotation J5 |

### Logs

Le programme genere des logs detailles dans la console :

```
[INFO] [BarreDetectionNode]: === PHASE DE DESCENTE (sans camera) ===
[INFO] [BarreDetectionNode]: Distance totale: 150.0 mm
[INFO] [BarreDetectionNode]:   J2: -100.0 mm (2/3)
[INFO] [BarreDetectionNode]:   J4: -50.0 mm (1/3)
[INFO] [BarreDetectionNode]:  J0 (horizontal): +32.8 mm
[INFO] [BarreDetectionNode]: Compensation J3 (vertical): +7.8 mm
```

### Architecture

```
BarreDetectionNode
+-- ROS Interface
|   +-- Publishers (/arm/joint_position, /wrist/joint_position)
|   +-- Subscribers (/arm/joint_state, /wrist/joint_state)
+-- Vision
|   +-- Detection du cadre (masques HSV)
|   +-- Warp perspective
|   +-- Detection des barres (projection horizontale)
+-- Mouvements
|   +-- Alignements (J0, J1, J3)
|   +-- Descente calculee
|   +-- Navigation
|   +-- Inspection
+-- Interface Utilisateur
    +-- Commandes clavier
    +-- Fichiers de commandes
```
