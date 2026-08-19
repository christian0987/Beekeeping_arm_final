#!/usr/bin/env python3
"""
Outil de visualisation HSV avancé pour ajuster les masques.
Selectionnez une zone avec la souris pour voir les valeurs HSV précises.

Fonctionnalités :
- Survol : affiche les valeurs HSV du pixel
- Selection d'une zone : affiche les valeurs min, max, moyenne de la zone
- Trackbars pour ajuster le masque en temps réel
- Sauvegarde automatique des valeurs avec 's'
- Chargement automatique des dernières valeurs sauvegardées
- Plusieurs profils de couleurs (orange, jaune, blanc, beige, personnalisé)

Touches :
- 'q' : Quitter
- 's' : Sauvegarder les valeurs actuelles
- '1' : Profil Orange
- '2' : Profil Jaune
- '3' : Profil Blanc
- '4' : Profil Beige
- '5' : Profil Personnalisé
- Clic gauche : Démarrer la sélection d'une zone
- Relâcher : Afficher les statistiques HSV de la zone
"""

import cv2
import numpy as np
import json
import os

# ============================================================================
# PROFILS DE COULEUR
# ============================================================================

PROFILS = {
    'orange': {
        'nom': 'Orange',
        'H_MIN': 0, 'H_MAX': 20,
        'S_MIN': 40, 'S_MAX': 255,
        'V_MIN': 40, 'V_MAX': 255
    },
    'jaune': {
        'nom': 'Jaune',
        'H_MIN': 20, 'H_MAX': 60,
        'S_MIN': 30, 'S_MAX': 255,
        'V_MIN': 30, 'V_MAX': 255
    },
    'blanc': {
        'nom': 'Blanc',
        'H_MIN': 0, 'H_MAX': 180,
        'S_MIN': 0, 'S_MAX': 30,
        'V_MIN': 200, 'V_MAX': 255
    },
    'beige': {
        'nom': 'Beige',
        'H_MIN': 10, 'H_MAX': 35,
        'S_MIN': 30, 'S_MAX': 90,
        'V_MIN': 120, 'V_MAX': 220
    },
    'personnalise': {
        'nom': 'Personnalisé',
        'H_MIN': 0, 'H_MAX': 180,
        'S_MIN': 0, 'S_MAX': 255,
        'V_MIN': 0, 'V_MAX': 255
    }
}

# Fichier de sauvegarde
FICHIER_SAUVEGARDE = "hsv_profils.json"

# ============================================================================
# CHARGEMENT/SAUVEGARDE DES PROFILS
# ============================================================================

def charger_profils():
    """Charge les profils sauvegardés depuis un fichier."""
    if os.path.exists(FICHIER_SAUVEGARDE):
        try:
            with open(FICHIER_SAUVEGARDE, 'r') as f:
                data = json.load(f)
            print(f"✅ Profils chargés depuis {FICHIER_SAUVEGARDE}")
            return data
        except:
            print(f"⚠️ Erreur de lecture du fichier {FICHIER_SAUVEGARDE}")
            return PROFILS
    return PROFILS

def sauvegarder_profils(profils):
    """Sauvegarde les profils dans un fichier."""
    try:
        with open(FICHIER_SAUVEGARDE, 'w') as f:
            json.dump(profils, f, indent=4)
        print(f"✅ Profils sauvegardés dans {FICHIER_SAUVEGARDE}")
        return True
    except:
        print(f"❌ Erreur de sauvegarde dans {FICHIER_SAUVEGARDE}")
        return False

# ============================================================================
# FONCTIONS
# ============================================================================

# Variables globales pour la sélection
selection_en_cours = False
x1, y1, x2, y2 = 0, 0, 0, 0

def rien(x):
    """Fonction vide pour les trackbars."""
    pass

def afficher_valeurs_hsv(event, x, y, flags, param):
    """Gère les événements souris pour la sélection de zone."""
    global selection_en_cours, x1, y1, x2, y2
    
    if event == cv2.EVENT_LBUTTONDOWN:
        selection_en_cours = True
        x1, y1 = x, y
        x2, y2 = x, y
        
    elif event == cv2.EVENT_MOUSEMOVE:
        if selection_en_cours:
            x2, y2 = x, y
        else:
            # Afficher les valeurs du pixel survolé
            hsv = param['hsv']
            h, s, v = hsv[y, x]
            print(f"HSV à ({x}, {y}) : H={h:3d}, S={s:3d}, V={v:3d}", end='\r')
            
    elif event == cv2.EVENT_LBUTTONUP:
        selection_en_cours = False
        x2, y2 = x, y
        # Afficher les statistiques de la zone
        afficher_stats_zone(param['frame'], param['hsv'], x1, y1, x2, y2, param['profils'])

def afficher_stats_zone(frame, hsv, x1, y1, x2, y2, profils):
    """Affiche les statistiques HSV de la zone sélectionnée."""
    x_min = min(x1, x2)
    x_max = max(x1, x2)
    y_min = min(y1, y2)
    y_max = max(y1, y2)
    
    zone = hsv[y_min:y_max, x_min:x_max]
    if zone.size == 0:
        return
    
    h_min = np.min(zone[:, :, 0])
    h_max = np.max(zone[:, :, 0])
    h_mean = np.mean(zone[:, :, 0])
    
    s_min = np.min(zone[:, :, 1])
    s_max = np.max(zone[:, :, 1])
    s_mean = np.mean(zone[:, :, 1])
    
    v_min = np.min(zone[:, :, 2])
    v_max = np.max(zone[:, :, 2])
    v_mean = np.mean(zone[:, :, 2])
    
    print("\n" + "=" * 70)
    print("📊 STATISTIQUES HSV DE LA ZONE SELECTIONNEE")
    print("=" * 70)
    print(f"Zone : ({x_min}, {y_min}) -> ({x_max}, {y_max})")
    print(f"Taille : {zone.shape[0]} x {zone.shape[1]} pixels")
    print("-" * 70)
    print(f"  H : min={h_min:3d}  max={h_max:3d}  moyenne={h_mean:.1f}  écart={(h_max-h_min):3d}")
    print(f"  S : min={s_min:3d}  max={s_max:3d}  moyenne={s_mean:.1f}  écart={(s_max-s_min):3d}")
    print(f"  V : min={v_min:3d}  max={v_max:3d}  moyenne={v_mean:.1f}  écart={(v_max-v_min):3d}")
    print("-" * 70)
    print("\n💡 SUGGESTIONS DE VALEURS POUR LE MASQUE :")
    print("-" * 70)
    
    marge_h = max(5, (h_max - h_min) // 4)
    marge_s = max(5, (s_max - s_min) // 4)
    marge_v = max(5, (v_max - v_min) // 4)
    
    h_min_sugg = max(0, h_min - marge_h)
    h_max_sugg = min(180, h_max + marge_h)
    s_min_sugg = max(0, s_min - marge_s)
    s_max_sugg = min(255, s_max + marge_s)
    v_min_sugg = max(0, v_min - marge_v)
    v_max_sugg = min(255, v_max + marge_v)
    
    print(f"  np.array([{h_min_sugg}, {s_min_sugg}, {v_min_sugg}])")
    print(f"  np.array([{h_max_sugg}, {s_max_sugg}, {v_max_sugg}])")
    print("-" * 70)
    print("(Ajustez les trackbars pour affiner)")
    print("=" * 70)
    print()

def appliquer_profil(profils, nom_profil):
    """Applique un profil aux trackbars."""
    if nom_profil not in profils:
        return
    
    p = profils[nom_profil]
    cv2.setTrackbarPos("H min", "Controles HSV", p['H_MIN'])
    cv2.setTrackbarPos("H max", "Controles HSV", p['H_MAX'])
    cv2.setTrackbarPos("S min", "Controles HSV", p['S_MIN'])
    cv2.setTrackbarPos("S max", "Controles HSV", p['S_MAX'])
    cv2.setTrackbarPos("V min", "Controles HSV", p['V_MIN'])
    cv2.setTrackbarPos("V max", "Controles HSV", p['V_MAX'])
    
    print(f"✅ Profil '{p['nom']}' appliqué")
    print(f"   H: {p['H_MIN']:3d} - {p['H_MAX']:3d}")
    print(f"   S: {p['S_MIN']:3d} - {p['S_MAX']:3d}")
    print(f"   V: {p['V_MIN']:3d} - {p['V_MAX']:3d}")

def sauvegarder_profil_actuel(profils, h_min, h_max, s_min, s_max, v_min, v_max):
    """Sauvegarde le profil actuel comme 'personnalise'."""
    profils['personnalise'] = {
        'nom': 'Personnalisé',
        'H_MIN': h_min,
        'H_MAX': h_max,
        'S_MIN': s_min,
        'S_MAX': s_max,
        'V_MIN': v_min,
        'V_MAX': v_max
    }
    
    if sauvegarder_profils(profils):
        print("\n📋 VALEURS SAUVEGARDEES :")
        print(f"  H: {h_min:3d} - {h_max:3d}")
        print(f"  S: {s_min:3d} - {s_max:3d}")
        print(f"  V: {v_min:3d} - {v_max:3d}")
        print("\n📋 À COPIER DANS VOTRE CODE :")
        print("-" * 40)
        print(f"BAS  = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"HAUT = np.array([{h_max}, {s_max}, {v_max}])")
        print("-" * 40)

# ============================================================================
# PROGRAMME PRINCIPAL
# ============================================================================

def main():
    global selection_en_cours, x1, y1, x2, y2
    
    # Charger les profils
    profils = charger_profils()
    
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("❌ Aucune caméra trouvée")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow("Image originale", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Masque HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Controles HSV", cv2.WINDOW_NORMAL)

    # Créer les trackbars
    cv2.createTrackbar("H min", "Controles HSV", 0, 180, rien)
    cv2.createTrackbar("H max", "Controles HSV", 180, 180, rien)
    cv2.createTrackbar("S min", "Controles HSV", 0, 255, rien)
    cv2.createTrackbar("S max", "Controles HSV", 255, 255, rien)
    cv2.createTrackbar("V min", "Controles HSV", 0, 255, rien)
    cv2.createTrackbar("V max", "Controles HSV", 255, 255, rien)

    print("=" * 70)
    print("🖱️  OUTIL DE VISUALISATION HSV AVANCÉ")
    print("=" * 70)
    print("  • Survoler : voir les valeurs HSV du pixel")
    print("  • Clic et glisser : sélectionner une zone")
    print("  • Relâcher : voir les statistiques de la zone")
    print("  • Ajuster les trackbars : voir le masque en temps réel")
    print("=" * 70)
    print("\n📌 PROFILS DE COULEUR :")
    print("  '1' : Orange     '2' : Jaune     '3' : Blanc")
    print("  '4' : Beige      '5' : Personnalisé")
    print("  's' : Sauvegarder les valeurs actuelles")
    print("  'q' : Quitter")
    print("=" * 70)

    # Appliquer le dernier profil utilisé ou le blanc par défaut
    appliquer_profil(profils, 'blanc')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H min", "Controles HSV")
        h_max = cv2.getTrackbarPos("H max", "Controles HSV")
        s_min = cv2.getTrackbarPos("S min", "Controles HSV")
        s_max = cv2.getTrackbarPos("S max", "Controles HSV")
        v_min = cv2.getTrackbarPos("V min", "Controles HSV")
        v_max = cv2.getTrackbarPos("V max", "Controles HSV")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        frame_display = frame.copy()
        if selection_en_cours:
            cv2.rectangle(frame_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_display, f"({x1},{y1}) -> ({x2},{y2})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Image originale", frame_display)
        cv2.imshow("Masque HSV", masked)

        cv2.setMouseCallback("Image originale", afficher_valeurs_hsv, 
                            {'hsv': hsv, 'frame': frame, 'profils': profils})

        # Afficher les valeurs sur le masque
        cv2.putText(masked, f"H: {h_min:3d} - {h_max:3d}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(masked, f"S: {s_min:3d} - {s_max:3d}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(masked, f"V: {v_min:3d} - {v_max:3d}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        nb_pixels = cv2.countNonZero(mask)
        pourcentage = (nb_pixels / (frame.shape[0] * frame.shape[1])) * 100
        cv2.putText(masked, f"Pixels: {nb_pixels} ({pourcentage:.1f}%)", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            sauvegarder_profil_actuel(profils, h_min, h_max, s_min, s_max, v_min, v_max)
        elif key == ord('1'):
            appliquer_profil(profils, 'orange')
        elif key == ord('2'):
            appliquer_profil(profils, 'jaune')
        elif key == ord('3'):
            appliquer_profil(profils, 'blanc')
        elif key == ord('4'):
            appliquer_profil(profils, 'beige')
        elif key == ord('5'):
            appliquer_profil(profils, 'personnalise')

    cap.release()
    cv2.destroyAllWindows()
    print("\nFin")

if __name__ == '__main__':
    main()