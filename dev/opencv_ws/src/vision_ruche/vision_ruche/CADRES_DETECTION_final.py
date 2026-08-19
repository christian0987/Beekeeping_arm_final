#!/usr/bin/env python3
"""
Detection autonome des barres + Inspection.
Sequence complete :
1. Detection du cadre + alignements iteratifs (J0 <-> J1) + J3
2. Descente à 2 cm du cadre
3. Pour chaque barre demandée : aller à la barre + inspection (J2 puis J5)
4. Fin : tous les joints à 0
"""

import cv2
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ============================================================================
# PARAMETRES
# ============================================================================

# --- Detection du rectangle exterieur (couvercle) ---
CADRE_ORANGE_BAS = np.array([0, 40, 40])
CADRE_ORANGE_HAUT = np.array([20, 255, 255])

CADRE_JAUNE_BAS = np.array([20, 30, 30])
CADRE_JAUNE_HAUT = np.array([60, 255, 255])

CADRE_BLANC_BAS = np.array([0, 0, 169])
CADRE_BLANC_HAUT = np.array([99, 95, 255])

CADRE_BEIGE_BAS = np.array([10, 30, 120])
CADRE_BEIGE_HAUT = np.array([35, 90, 220])

# --- Seuils pour les BARRES (utilise le masque BLANC) ---
BAS = np.array([0, 0, 173])
HAUT = np.array([92, 121, 255])

KERNEL_SIZE = (15, 15)
AREA_MIN_CADRE = 3000
POLY_EPSILON_FACTOR = 0.02

# Dimensions de l'image redressee
WARP_LARGEUR = 400
WARP_HAUTEUR = 600

# Parametres de detection par projection horizontale
RATIO_BARRE_MIN = 0.25
RATIO_ESPACE_MAX = 0.10
HAUTEUR_MIN_BARRE_PX = 3
HAUTEUR_MIN_ESPACE_PX = 2
ESPACEMENT_MIN_PX = 3
MARGE_BORD_PX = 5

# Parametres pour le remplissage des trous
TAILLE_KERNEL_HORIZONTAL = (15, 3)
TAILLE_KERNEL_VERTICAL = (3, 5)

# Dimensions reelles du cadre
LARGEUR_REELLE_MM = 510.0
HAUTEUR_REELLE_MM = 410.0

# Nombre de barres pour la memorisation automatique
NB_BARRES_AUTO_MEMO = 10

# ============================================================================
# VALEURS DE CALIBRATION
# ============================================================================
FACTEUR_CORRECTION = 0.7845 
CALIB_OFFSET_X = 0  # mm
CALIB_OFFSET_Y = 40   # mm
OFFSET_PINCE = 100  # mm Décalage entre la caméra et la pince
OFFSET_HAUTEUR_PINCE = 0  # mm - Décalage vertical caméra/pince
                            # Positif = pince plus basse que la caméra
                            # Négatif = pince plus haute que la caméra

# Constante de compensation (sinus exact)
SIN_ALPHA_X = 105.0 / 480.0  # = 0.21875 pour J0 (horizontal)
SIN_ALPHA_Y = 20 / 480.0   # = 0.05208 pour J3 (vertical)

# ============================================================================
# CONSTANTES POUR L'INSPECTION
# ============================================================================
HAUTEUR_REMONTEE_INSPECTION = 200.0  # mm - Hauteur de remontée de J2 pendant l'inspection
HAUTEUR_REMONTEE_ROTATION = 50.0     # mm - Hauteur de remontée pour la rotation J5
TEMPS_ATTENTE_INSPECTION = 5.0       # secondes - Temps d'attente à +90° et -90°
ANGLE_J5_POSITIF = 90.0              # degrés - Angle de rotation positive de J5
ANGLE_J5_NEGATIF = -90.0             # degrés - Angle de rotation négative de J5
ANGLE_J5_ROTATION = -150.0           # degrés - Angle de rotation pour réduire l'offset
TOLERANCE_J5 = 10.0                  # degrés - Tolérance pour la rotation J5
TOLERANCE_J5_ZERO = 0.5              # degrés - Tolérance pour le retour à 0 de J5
TOLERANCE_J4 = 0.5                   # mm - Tolérance pour J4
TOLERANCE_J2 = 0.5                   # mm - Tolérance pour J2

print(f"Facteur de correction : {FACTEUR_CORRECTION:.3f}")
print(f"   Offset X : {CALIB_OFFSET_X:.1f} mm")
print(f"   Offset Y : {CALIB_OFFSET_Y:.1f} mm")
print(f"   Offset Pince : {OFFSET_PINCE:.1f} mm")
print(f"   Offset Hauteur Pince : {OFFSET_HAUTEUR_PINCE:.1f} mm")
print(f"Compensation J0 (horizontal): sin(alpha) = {SIN_ALPHA_X:.4f}")
print(f"Compensation J3 (vertical):   sin(alpha) = {SIN_ALPHA_Y:.4f}")
print(f"Inspection : Remontée J2 = {HAUTEUR_REMONTEE_INSPECTION:.1f} mm")
print(f"             Remontée rotation = {HAUTEUR_REMONTEE_ROTATION:.1f} mm")

# ============================================================================
# FONCTIONS - DETECTION DU CADRE
# ============================================================================

def ordonner_points(pts):
    """
    Ordonne 4 points dans l'ordre : Haut-Gauche, Haut-Droit, Bas-Droit, Bas-Gauche.
    
    Args:
        pts (np.ndarray): Tableau de 4 points (4x2)
    
    Returns:
        np.ndarray: Points ordonnes (4x2)
    """
    centre = np.mean(pts, axis=0)
    def angle_from_centre(p):
        return math.atan2(p[1] - centre[1], p[0] - centre[0])
    angles = [angle_from_centre(p) for p in pts]
    idx = np.argsort(angles)
    sorted_pts = pts[idx]
    sums = [p[0] + p[1] for p in sorted_pts]
    idx_hg = np.argmin(sums)
    ordered = np.roll(sorted_pts, -idx_hg, axis=0)
    return ordered


def masque_cadre(image):
    """
    Cree un masque binaire pour detecter le cadre (orange, jaune, blanc, beige).
    
    Args:
        image (np.ndarray): Image BGR
    
    Returns:
        tuple: (mask, mask_orange, mask_jaune, mask_blanc, mask_beige)
               - mask: masque combine
               - les autres: masques individuels par couleur
    """
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask_orange = cv2.inRange(hsv, CADRE_ORANGE_BAS, CADRE_ORANGE_HAUT)
    mask_jaune = cv2.inRange(hsv, CADRE_JAUNE_BAS, CADRE_JAUNE_HAUT)
    mask_blanc = cv2.inRange(hsv, CADRE_BLANC_BAS, CADRE_BLANC_HAUT)
    mask_beige = cv2.inRange(hsv, CADRE_BEIGE_BAS, CADRE_BEIGE_HAUT)

    mask = cv2.bitwise_or(mask_orange, mask_jaune)
    mask = cv2.bitwise_or(mask, mask_blanc)
    mask = cv2.bitwise_or(mask, mask_beige)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, KERNEL_SIZE)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask, mask_orange, mask_jaune, mask_blanc, mask_beige


def plus_grand_quadrilatere(mask):
    """
    Trouve le plus grand quadrilatere dans un masque binaire.
    
    Args:
        mask (np.ndarray): Masque binaire
    
    Returns:
        tuple: (pts, contour) ou (None, None) si non trouve
               - pts: points du quadrilatere ordonnes (4x2)
               - contour: contour trouve
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < AREA_MIN_CADRE:
        return None, None
    peri = cv2.arcLength(c, True)
    eps = POLY_EPSILON_FACTOR * peri
    approx = cv2.approxPolyDP(c, eps, True)
    max_iter = 5
    while len(approx) != 4 and max_iter > 0:
        eps *= 1.2
        approx = cv2.approxPolyDP(c, eps, True)
        max_iter -= 1
    if len(approx) != 4:
        return None, None
    pts = approx.reshape(4, 2).astype(np.float32)
    pts = ordonner_points(pts)
    return pts.astype(np.int32), c


# ============================================================================
# FONCTIONS - WARP PERSPECTIVE
# ============================================================================

def calculer_homographies(pts_cadre):
    """
    Calcule les matrices de transformation pour le warp perspective.
    
    Args:
        pts_cadre (np.ndarray): Points du cadre (4x2)
    
    Returns:
        tuple: (M, M_inv)
               - M: matrice de transformation avant
               - M_inv: matrice de transformation inverse
    """
    src = pts_cadre.astype(np.float32)
    dst = np.array([
        [0, 0],
        [WARP_LARGEUR - 1, 0],
        [WARP_LARGEUR - 1, WARP_HAUTEUR - 1],
        [0, WARP_HAUTEUR - 1]
    ], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv


def redresser_image(frame, M):
    """
    Redresse une image par transformation perspective.
    
    Args:
        frame (np.ndarray): Image originale
        M (np.ndarray): Matrice de transformation
    
    Returns:
        np.ndarray: Image redressee
    """
    return cv2.warpPerspective(frame, M, (WARP_LARGEUR, WARP_HAUTEUR))


# ============================================================================
# FONCTIONS - DETECTION DES BARRES
# ============================================================================

def masque_blanc_warp(warp):
    """
    Cree un masque pour les barres blanches dans l'image redressee.
    
    Args:
        warp (np.ndarray): Image redressee
    
    Returns:
        np.ndarray: Masque binaire des barres
    """
    blurred = cv2.GaussianBlur(warp, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BAS, HAUT)

    kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, TAILLE_KERNEL_HORIZONTAL)
    mask_filled = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_horizontal)
    
    kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, TAILLE_KERNEL_VERTICAL)
    mask_filled = cv2.morphologyEx(mask_filled, cv2.MORPH_OPEN, kernel_vertical, iterations=1)
    mask_filled = cv2.morphologyEx(mask_filled, cv2.MORPH_CLOSE, kernel_vertical, iterations=1)
    
    kernel_nettoyage = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_filled = cv2.morphologyEx(mask_filled, cv2.MORPH_OPEN, kernel_nettoyage, iterations=1)
    mask_filled = cv2.morphologyEx(mask_filled, cv2.MORPH_CLOSE, kernel_nettoyage, iterations=1)

    return mask_filled


def detecter_barres_et_espaces(mask_warp):
    """
    Detecte les barres et espaces par projection horizontale.
    
    Args:
        mask_warp (np.ndarray): Masque binaire des barres
    
    Returns:
        tuple: (barres, ligne_ratio, espaces)
               - barres: liste des barres detectees
               - ligne_ratio: ratio de pixels blancs par ligne
               - espaces: liste des espaces detectes
    """
    h, w = mask_warp.shape[:2]
    ligne_ratio = np.sum(mask_warp > 0, axis=1).astype(np.float32) / float(w)
    ligne_ratio[:MARGE_BORD_PX] = 0.0
    ligne_ratio[h - MARGE_BORD_PX:] = 0.0

    est_barre = ligne_ratio > RATIO_BARRE_MIN
    est_espace = ligne_ratio < RATIO_ESPACE_MAX

    barres = []
    y = 0
    while y < h:
        if est_barre[y]:
            y_start = y
            while y < h and est_barre[y]:
                y += 1
            y_end = y - 1
            epaisseur = y_end - y_start + 1
            if epaisseur >= HAUTEUR_MIN_BARRE_PX:
                ratio_moyen = float(np.mean(ligne_ratio[y_start:y_end + 1]))
                barres.append({
                    'y_top': y_start,
                    'y_bottom': y_end,
                    'y_centre': (y_start + y_end) / 2.0,
                    'epaisseur': epaisseur,
                    'ratio_moyen': ratio_moyen
                })
        else:
            y += 1

    espaces = []
    y = 0
    while y < h:
        if est_espace[y]:
            y_start = y
            while y < h and est_espace[y]:
                y += 1
            y_end = y - 1
            epaisseur = y_end - y_start + 1
            if epaisseur >= HAUTEUR_MIN_ESPACE_PX:
                espaces.append({
                    'y_top': y_start,
                    'y_bottom': y_end,
                    'y_centre': (y_start + y_end) / 2.0,
                    'epaisseur': epaisseur
                })
        else:
            y += 1

    barres = fusionner_barres(barres)
    espaces = fusionner_espaces(espaces)

    if len(barres) > NB_BARRES_AUTO_MEMO:
        barres = barres[1:-1]

    barres = sorted(barres, key=lambda b: b['y_centre'])
    espaces = sorted(espaces, key=lambda e: e['y_centre'])
    
    return barres, ligne_ratio, espaces


def fusionner_barres(barres):
    """
    Fusionne les barres proches entre elles.
    
    Args:
        barres (list): Liste des barres
    
    Returns:
        list: Liste des barres fusionnees
    """
    if not barres:
        return []
    barres = sorted(barres, key=lambda b: b['y_top'])
    fusionnes = [barres[0]]
    for b in barres[1:]:
        dernier = fusionnes[-1]
        ecart = b['y_top'] - dernier['y_bottom']
        if ecart < ESPACEMENT_MIN_PX:
            dernier['y_bottom'] = max(dernier['y_bottom'], b['y_bottom'])
            dernier['y_top'] = min(dernier['y_top'], b['y_top'])
            dernier['y_centre'] = (dernier['y_top'] + dernier['y_bottom']) / 2.0
            dernier['epaisseur'] = dernier['y_bottom'] - dernier['y_top'] + 1
            dernier['ratio_moyen'] = max(dernier['ratio_moyen'], b['ratio_moyen'])
        else:
            fusionnes.append(b)
    return fusionnes


def fusionner_espaces(espaces):
    """
    Fusionne les espaces proches entre eux.
    
    Args:
        espaces (list): Liste des espaces
    
    Returns:
        list: Liste des espaces fusionnes
    """
    if not espaces:
        return []
    espaces = sorted(espaces, key=lambda e: e['y_top'])
    fusionnes = [espaces[0]]
    for e in espaces[1:]:
        dernier = fusionnes[-1]
        ecart = e['y_top'] - dernier['y_bottom']
        if ecart < ESPACEMENT_MIN_PX:
            dernier['y_bottom'] = max(dernier['y_bottom'], e['y_bottom'])
            dernier['y_top'] = min(dernier['y_top'], e['y_top'])
            dernier['y_centre'] = (dernier['y_top'] + dernier['y_bottom']) / 2.0
            dernier['epaisseur'] = dernier['y_bottom'] - dernier['y_top'] + 1
        else:
            fusionnes.append(e)
    return fusionnes


def barres_vers_image_originale(barres, M_inv):
    """
    Projette les barres de l'image redressee vers l'image originale.
    
    Args:
        barres (list): Liste des barres dans l'image redressee
        M_inv (np.ndarray): Matrice de transformation inverse
    
    Returns:
        list: Barres projetees dans l'image originale
    """
    barres_image = []
    for b in barres:
        pts_warp = np.array([
            [0, b['y_top']],
            [WARP_LARGEUR - 1, b['y_top']],
            [WARP_LARGEUR - 1, b['y_bottom']],
            [0, b['y_bottom']]
        ], dtype=np.float32).reshape(-1, 1, 2)

        pts_image = cv2.perspectiveTransform(pts_warp, M_inv).reshape(-1, 2)
        pts_image = pts_image.astype(np.int32)

        centre = pts_image.mean(axis=0).astype(int)

        barres_image.append({
            'polygone': pts_image,
            'centre': (int(centre[0]), int(centre[1])),
            'y_warp': b['y_centre'],
            'epaisseur_warp': b['epaisseur'],
            'ratio_moyen': b['ratio_moyen'],
            'y_top_warp': b['y_top'],
            'y_bottom_warp': b['y_bottom']
        })

    return barres_image


def calculer_coordonnees_barre(barre, pts_cadre, frame_shape, M_inv):
    """
    Calcule les coordonnees 3D (X, Y, Z) d'une barre.
    
    Args:
        barre (dict): Donnees de la barre
        pts_cadre (np.ndarray): Points du cadre
        frame_shape (tuple): Dimensions de l'image (h, w)
        M_inv (np.ndarray): Matrice de transformation inverse
    
    Returns:
        dict: Coordonnees 3D de la barre
              - x_mm: position X en mm
              - y_mm: position Y en mm  
              - z_mm: position Z en mm (distance)
              - centre_image: centre de l'image
              - echelle: echelle mm/px
    """
    h, w = frame_shape[:2]
    centre_x, centre_y = barre['centre']
    pts_cadre_int = pts_cadre.astype(np.int32)
    largeur_cadre_px = np.linalg.norm(pts_cadre_int[1] - pts_cadre_int[0])
    focale_px = 500.0
    z_mm_brut = (LARGEUR_REELLE_MM * focale_px) / largeur_cadre_px if largeur_cadre_px > 0 else 0
    z_mm = z_mm_brut * FACTEUR_CORRECTION
    centre_image_x = w // 2
    centre_image_y = h // 2
    echelle_mm_par_px = z_mm / focale_px
    x_mm = (centre_x - centre_image_x) * echelle_mm_par_px - CALIB_OFFSET_X
    y_mm = (centre_y - centre_image_y) * echelle_mm_par_px - CALIB_OFFSET_Y
    return {
        'x_mm': x_mm,
        'y_mm': y_mm,
        'z_mm': z_mm,
        'centre_image': (centre_image_x, centre_image_y),
        'echelle': echelle_mm_par_px
    }


# ============================================================================
# FONCTIONS - AFFICHAGE
# ============================================================================

def dessiner_axes(image, origine, longueur=80, echelle=1.0):
    """
    Dessine les axes X, Y sur une image.
    
    Args:
        image (np.ndarray): Image sur laquelle dessiner
        origine (tuple): Point d'origine (x, y)
        longueur (int): Longueur des axes en pixels
        echelle (float): Facteur d'echelle
    """
    cx, cy = origine
    l = int(longueur * echelle)
    
    cv2.arrowedLine(image, (cx, cy), (cx + l, cy), (0, 0, 255), 2, tipLength=0.2)
    cv2.putText(image, "X", (cx + l + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.arrowedLine(image, (cx, cy), (cx, cy + l), (0, 255, 0), 2, tipLength=0.2)
    cv2.putText(image, "Y", (cx + 5, cy + l + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.arrowedLine(image, (cx, cy), (cx - l//2, cy), (100, 100, 100), 1, tipLength=0.2)
    cv2.putText(image, "-X", (cx - l//2 - 20, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
    cv2.arrowedLine(image, (cx, cy), (cx, cy - l//2), (100, 100, 100), 1, tipLength=0.2)
    cv2.putText(image, "-Y", (cx + 5, cy - l//2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
    cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)
    cv2.circle(image, (cx, cy), 7, (255, 255, 255), 1)


def image_vide(largeur, hauteur, message=""):
    """
    Cree une image vide avec un message optionnel.
    
    Args:
        largeur (int): Largeur de l'image
        hauteur (int): Hauteur de l'image
        message (str): Message a afficher
    
    Returns:
        np.ndarray: Image vide
    """
    img = np.zeros((hauteur, largeur, 3), dtype=np.uint8)
    if message:
        cv2.putText(img, message, (10, hauteur // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
    return img


def dessiner_resultat(image, pts_cadre, barres_image, roi_memorise, coords_premiere_barre=None, 
                      nb_barres_total=0, auto_memo=False, mise_a_jour=False, derniere_verification=0,
                      nb_barres_actuel=0, nb_barres_memo=0, barres_figees=None, pts_figes=None,
                      en_attente=False):
    """
    Dessine le resultat de la detection sur l'image.
    
    Args:
        image (np.ndarray): Image originale
        pts_cadre (np.ndarray): Points du cadre
        barres_image (list): Barres detectees
        roi_memorise (bool): Si la ROI est memorisee
        coords_premiere_barre (dict): Coordonnees de la premiere barre
        nb_barres_total (int): Nombre total de barres
        auto_memo (bool): Si le figage auto est actif
        mise_a_jour (bool): Si une mise a jour a ete effectuee
        derniere_verification (float): Timestamp de la derniere verification
        nb_barres_actuel (int): Nombre de barres actuellement detectees
        nb_barres_memo (int): Nombre de barres memorisees
        barres_figees (list): Barres figees
        pts_figes (np.ndarray): Points du cadre figes
        en_attente (bool): Si on attend le nombre exact de barres
    
    Returns:
        np.ndarray: Image avec les dessins
    """
    result = image.copy()
    h, w = result.shape[:2]
    centre_cam_x = w // 2
    centre_cam_y = h // 2
    
    dessiner_axes(result, (centre_cam_x, centre_cam_y), longueur=100, echelle=1.0)
    cv2.putText(result, "ORIGINE", (centre_cam_x + 15, centre_cam_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    if barres_figees is not None and pts_figes is not None:
        cv2.polylines(result, [pts_figes], True, (0, 255, 0), 3)
        centre = np.mean(pts_figes, axis=0).astype(int)
        cv2.putText(result, "ROI FIGEE (10 barres)", (centre[0] - 60, centre[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        for i, barre in enumerate(barres_figees):
            cv2.polylines(result, [barre['polygone']], True, (0, 0, 255), 2)
            cv2.circle(result, barre['centre'], 4, (0, 255, 255), -1)
            cv2.putText(result, f"#{i+1}", (barre['centre'][0] - 15, barre['centre'][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(result, f"y={barre['y_warp']:.0f}", 
                        (barre['centre'][0] - 15, barre['centre'][1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
        
        for barre in barres_figees:
            centre_x, centre_y = barre['centre']
            cv2.line(result, (0, centre_y), (result.shape[1], centre_y), (255, 0, 0), 1)
        
        cv2.putText(result, f"Barres: {len(barres_figees)} (FIGEES)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        if nb_barres_actuel != NB_BARRES_AUTO_MEMO and en_attente:
            if nb_barres_actuel < NB_BARRES_AUTO_MEMO:
                msg = f"(Actuellement: {nb_barres_actuel} barres - moins que {NB_BARRES_AUTO_MEMO})"
            else:
                msg = f"(Actuellement: {nb_barres_actuel} barres - plus que {NB_BARRES_AUTO_MEMO})"
            cv2.putText(result, msg, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
            cv2.putText(result, f"En attente de EXACTEMENT {NB_BARRES_AUTO_MEMO} barres...", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
        elif nb_barres_actuel == NB_BARRES_AUTO_MEMO:
            cv2.putText(result, f"(Detection actuelle: {nb_barres_actuel} barres - OK)", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
            if mise_a_jour:
                cv2.putText(result, "Mise a jour effectuee", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
    
    elif pts_cadre is not None and not roi_memorise:
        cv2.polylines(result, [pts_cadre], True, (255, 255, 0), 2)
        
        barres_inversees = list(reversed(barres_image))
        for i, barre in enumerate(barres_inversees):
            num_barre = i + 1
            cv2.polylines(result, [barre['polygone']], True, (0, 0, 255), 2)
            cv2.circle(result, barre['centre'], 4, (0, 255, 255), -1)
            cv2.putText(result, f"#{num_barre}", (barre['centre'][0] - 15, barre['centre'][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(result, f"y={barre['y_warp']:.0f}", 
                        (barre['centre'][0] - 15, barre['centre'][1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
        
        for barre in barres_inversees:
            centre_x, centre_y = barre['centre']
            cv2.line(result, (0, centre_y), (result.shape[1], centre_y), (255, 0, 0), 1)
        
        if nb_barres_actuel == NB_BARRES_AUTO_MEMO:
            cv2.putText(result, f"Barres: {nb_barres_actuel} (OK - figage auto)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(result, "Figage automatique en cours...", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        else:
            cv2.putText(result, f"Barres: {nb_barres_actuel} (attendu: {NB_BARRES_AUTO_MEMO})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    y_offset = 80
    cv2.putText(result, "Axes:", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(result, "Rouge = X", (10, y_offset + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    cv2.putText(result, "Vert = Y", (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
    cv2.putText(result, "Traits:", (10, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    cv2.putText(result, "Rouge = centre camera", (10, y_offset + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    cv2.putText(result, "Bleu = centre barre", (10, y_offset + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
    
    if roi_memorise and barres_figees is not None:
        cv2.putText(result, "STATUT: FIGE (AUTO)", (10, y_offset + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(result, f"ROI figee avec {nb_barres_memo} barres", (10, y_offset + 125), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
    elif roi_memorise:
        cv2.putText(result, "STATUT: MEMORISEE", (10, y_offset + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(result, "STATUT: DETECTION", (10, y_offset + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(result, f"(Requis: {NB_BARRES_AUTO_MEMO} barres)", (10, y_offset + 125), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
    
    if coords_premiere_barre is not None:
        overlay = result.copy()
        cv2.rectangle(overlay, (w - 230, 10), (w - 10, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, result, 0.3, 0, result)
        
        nb_barres_affichees = len(barres_figees) if barres_figees is not None else nb_barres_total
        cv2.putText(result, f"Barre #1 (sur {nb_barres_affichees})", (w - 220, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        x_val = coords_premiere_barre['x_mm']
        y_val = coords_premiere_barre['y_mm']
        z_val = coords_premiere_barre['z_mm']
        
        cv2.putText(result, f"X: {x_val:6.1f} mm", (w - 220, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.putText(result, f"Y: {y_val:6.1f} mm", (w - 220, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(result, f"Z: {z_val:6.1f} mm", (w - 220, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)
        cv2.line(result, (w - 230, 105), (w - 10, 105), (100, 100, 100), 1)
        cv2.putText(result, "par rapport au centre", (w - 220, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
        cv2.putText(result, "de la camera", (w - 220, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
        cv2.putText(result, f"Echelle: {coords_premiere_barre['echelle']:.2f} mm/px", (w - 220, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)

    cv2.putText(result, "h:help  f:Fichier  b:ANNULER FIGE  d:Debug  c:Coords  a:AlignY  g:AlignAng  v:AlignCenter  s:Sequence  q:Quitter",
                (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return result


def dessiner_warp_avec_barres(warp, barres, espaces):
    """
    Dessine l'image redressee avec les barres et espaces detectes.
    
    Args:
        warp (np.ndarray): Image redressee
        barres (list): Liste des barres
        espaces (list): Liste des espaces
    
    Returns:
        np.ndarray: Image avec les dessins
    """
    disp = warp.copy()
    for b in barres:
        cv2.rectangle(disp, (0, b['y_top']), (WARP_LARGEUR - 1, b['y_bottom']), (0, 0, 255), 2)
        cv2.putText(disp, f"{b['ratio_moyen']*100:.0f}%", (5, max(15, b['y_top'] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    for e in espaces:
        cv2.rectangle(disp, (0, e['y_top']), (WARP_LARGEUR - 1, e['y_bottom']), (255, 0, 0), 2)
        cv2.putText(disp, "ESPACE", (5, max(15, e['y_top'] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
    cv2.putText(disp, f"Barres: {len(barres)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(disp, f"Espaces: {len(espaces)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    return disp


def dessiner_histogramme(ligne_ratio, hauteur_img, largeur_hist=150):
    """
    Dessine l'histogramme des ratios de pixels blancs par ligne.
    
    Args:
        ligne_ratio (np.ndarray): Ratio de pixels blancs par ligne
        hauteur_img (int): Hauteur de l'image
        largeur_hist (int): Largeur de l'histogramme
    
    Returns:
        np.ndarray: Image de l'histogramme
    """
    h = len(ligne_ratio)
    disp = np.zeros((h, largeur_hist, 3), dtype=np.uint8)
    for y in range(h):
        longueur = int(ligne_ratio[y] * largeur_hist)
        if ligne_ratio[y] > RATIO_BARRE_MIN:
            couleur = (0, 0, 255)
        elif ligne_ratio[y] < RATIO_ESPACE_MAX:
            couleur = (255, 0, 0)
        else:
            couleur = (80, 80, 80)
        cv2.line(disp, (0, y), (longueur, y), couleur, 1)
    
    seuil_barre_x = int(RATIO_BARRE_MIN * largeur_hist)
    seuil_espace_x = int(RATIO_ESPACE_MAX * largeur_hist)
    cv2.line(disp, (seuil_barre_x, 0), (seuil_barre_x, h - 1), (0, 255, 255), 1)
    cv2.line(disp, (seuil_espace_x, 0), (seuil_espace_x, h - 1), (255, 0, 255), 1)
    cv2.putText(disp, "BARRE", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    cv2.putText(disp, "ESPACE", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
    return disp


# ============================================================================
# CLASSE ROS NODE
# ============================================================================

class BarreDetectionNode(Node):
    """
    Noeud ROS pour la detection des barres et l'inspection.
    
    Cette classe gere :
    - La detection du cadre et des barres par camera
    - Les alignements automatiques (J0, J1, J3)
    - La descente a 2 cm du cadre
    - La navigation vers les barres
    - L'inspection des barres (rotation J5)
    - Le retour a la position home
    """
    
    def __init__(self):
        """Initialise le noeud ROS et les parametres."""
        super().__init__('barre_detection')
        
        # Publishers / Subscribers
        self.arm_pub = self.create_publisher(Float64MultiArray, 'arm/joint_position', 10)
        self.wrist_pub = self.create_publisher(Float64MultiArray, 'wrist/joint_position', 10)
        self.arm_state_sub = self.create_subscription(Float64MultiArray, 'arm/joint_state', self.arm_state_callback, 10)
        self.wrist_state_sub = self.create_subscription(Float64MultiArray, 'wrist/joint_state', self.wrist_state_callback, 10)
        
        self.current_arm_pos = None
        self.current_wrist_pos = None
        self.positions_received = False
        
        # Limites de securite
        self.arm_limits = {
            'j0': (-325.0, 325.0),
            'j1': (-45.0, 180.0),
            'j2': (-430.0, 370.0),
        }
        self.wrist_limits = {
            'j3': (-25.0, 85.0),
            'j4': (-240.0, 50.0),
            'j5': (-180.0, 180.0),
        }
        
        # Variables de detection
        self.roi_points = None
        self.roi_memorise = False
        self.nb_barres_memo = 0
        self.barres_figees = None
        self.pts_figes = None
        self.coords_figees = None
        self.auto_memo_active = False
        self.mise_a_jour_effectuee = False
        self.en_attente = False
        self.derniere_mise_a_jour = time.time()
        
        self.debug_mode = False
        self.show_coords = False
        
        # Variables pour la descente
        self.z_cam_actuel = None
        
        # Constantes de compensation
        self.SIN_ALPHA_X = 105.0 / 480.0
        self.SIN_ALPHA_Y = 20.0 / 480.0
        
        # Offset pince
        self.OFFSET_PINCE = 100  
        
        self.OFFSET_HAUTEUR_PINCE = -5  # mm - Décalage vertical caméra/pince
    
        # Positions des barres (offsets relatifs au centre du cadre)
        self.barres_offsets = {
            1: -160.0,
            2: -125.0,
            3: -90.0,
            4: -55.0,
            5: -20.0,
            6: 15.0,
            7: 50.0,
            8: 85.0,
            9: 120.0,
            10: 155.0
        }
        
        # Liste des barres a visiter (sera definie par l'utilisateur)
        self.liste_barres = [1, 2, 3]  # Valeur par defaut
        
        # J0 centre (apres alignement)
        self.j0_centre = 0.0
        
        # Constantes d'inspection
        self.HAUTEUR_REMONTEE_INSPECTION = 200.0
        self.HAUTEUR_REMONTEE_ROTATION = 50.0
        self.TEMPS_ATTENTE_INSPECTION = 5.0
        self.ANGLE_J5_POSITIF = 90.0
        self.ANGLE_J5_NEGATIF = -90.0
        self.ANGLE_J5_ROTATION = -150.0
        self.TOLERANCE_J5 = 10.0
        self.TOLERANCE_J5_ZERO = 0.5
        self.TOLERANCE_J4 = 0.5
        self.TOLERANCE_J2 = 0.5
        
        self.get_logger().info("BarreDetectionNode pret")
        self.get_logger().info(f"Compensation J0 (horizontal): sin(alpha) = {self.SIN_ALPHA_X:.4f}")
        self.get_logger().info(f"Compensation J3 (vertical):   sin(alpha) = {self.SIN_ALPHA_Y:.4f}")
        self.get_logger().info(f"Offset pince: {self.OFFSET_PINCE:.1f} mm")
        self.get_logger().info(f"Inspection : Remontée J2 = {self.HAUTEUR_REMONTEE_INSPECTION:.1f} mm")
        
        self.roi_log_counter = 0
        self.roi_log_interval = 50
    
    # ============================================================
    # NOUVELLE FONCTION : Mesure Z depuis le cadre
    # ============================================================
    def calculer_z_depuis_cadre(self, pts_cadre, frame_shape):
        """
        Calcule la distance Z à partir des dimensions du cadre dans l'image.
        
        Args:
            pts_cadre (np.ndarray): Points du cadre (4x2)
            frame_shape (tuple): Dimensions de l'image (h, w)
        
        Returns:
            float: Distance Z en mm, ou None si impossible
        """
        if pts_cadre is None or len(pts_cadre) != 4:
            return None
        
        h, w = frame_shape[:2]
        pts_cadre_int = pts_cadre.astype(np.int32)
        
        # Calculer la largeur du cadre en pixels (bord supérieur)
        largeur_cadre_px = np.linalg.norm(pts_cadre_int[1] - pts_cadre_int[0])
        
        # Calculer la hauteur du cadre en pixels (bord gauche)
        hauteur_cadre_px = np.linalg.norm(pts_cadre_int[3] - pts_cadre_int[0])
        
        # Focale estimée (peut être calibrée)
        focale_px = 500.0
        
        # Dimensions réelles du cadre
        LARGEUR_REELLE_MM = 510.0
        HAUTEUR_REELLE_MM = 410.0
        
        # Calcul de Z à partir de la largeur
        if largeur_cadre_px > 0:
            z_mm_brut = (LARGEUR_REELLE_MM * focale_px) / largeur_cadre_px
            z_mm = z_mm_brut * FACTEUR_CORRECTION
            
            # Vérification avec la hauteur (pour validation)
            if hauteur_cadre_px > 0:
                z_hauteur_brut = (HAUTEUR_REELLE_MM * focale_px) / hauteur_cadre_px
                z_hauteur = z_hauteur_brut * FACTEUR_CORRECTION
                
                # Si les deux valeurs sont proches, on prend la moyenne
                if abs(z_mm - z_hauteur) < 50.0:
                    z_mm = (z_mm + z_hauteur) / 2.0
            
            return z_mm
        
        return None
    
    # ============================================================
    # NOUVELLE FONCTION : Diagnostic CHECK
    # ============================================================
    def afficher_diagnostic(self):
        """Affiche un diagnostic complet du système."""
        self.get_logger().info("   DIAGNOSTIC COMPLET :")
        self.get_logger().info("  ─────────────────────")
        
        # Vérifier Z
        if self.z_cam_actuel is not None:
            self.get_logger().info(f"   Z mesuré : {self.z_cam_actuel:.1f} mm")
        else:
            self.get_logger().error("   Z NON mesuré !")
        
        # Vérifier J0 centre
        if self.j0_centre is not None:
            self.get_logger().info(f"   J0 centre : {self.j0_centre:.1f} mm")
        else:
            self.get_logger().error("   J0 centre NON enregistré !")
        
        # Vérifier les barres
        if self.barres_figees is not None:
            self.get_logger().info(f"   {len(self.barres_figees)} barres figées")
        else:
            self.get_logger().warn("   Aucune barre figée (pas grave si Z est mesuré)")
        
        # Vérifier J2
        if self.current_arm_pos is not None:
            j2 = self.current_arm_pos[2]
            if j2 < -10.0:
                self.get_logger().info(f"   J2 descendu : {j2:.1f} mm")
            else:
                self.get_logger().warn(f"   J2 NON descendu : {j2:.1f} mm")
        
        # Afficher les positions
        if self.current_arm_pos is not None and self.current_wrist_pos is not None:
            self.get_logger().info(f"   Positions : J0={self.current_arm_pos[0]:.1f}, J1={self.current_arm_pos[1]:.1f}, J2={self.current_arm_pos[2]:.1f}")
            self.get_logger().info(f"              J3={self.current_wrist_pos[0]:.1f}, J4={self.current_wrist_pos[1]:.1f}, J5={self.current_wrist_pos[2]:.1f}")
        
        # Conclusion
        self.get_logger().info("  ─────────────────────")
        if self.z_cam_actuel is None:
            self.get_logger().warn("   Z non mesuré → NE PAS FAIRE B")
            self.get_logger().warn("  → Faites A pour aligner et mesurer Z")
        elif self.current_arm_pos is not None and self.current_arm_pos[2] > -10.0:
            self.get_logger().warn("   J2 non descendu → Faites D 2")
        else:
            self.get_logger().info("   Tout est OK → Vous pouvez faire B")
    
    # ============================================================
    # Callbacks ROS
    # ============================================================
    def arm_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_arm_pos = [msg.data[0], msg.data[1], msg.data[2]]
            self.check_positions_received()
    
    def wrist_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_wrist_pos = [msg.data[0], msg.data[1], msg.data[2]]
            self.check_positions_received()
    
    def check_positions_received(self):
        if self.current_arm_pos is not None and self.current_wrist_pos is not None:
            self.positions_received = True
    
    # ============================================================
    # FONCTIONS D'ATTENTE ROBUSTES
    # ============================================================
    def wait_for_joint(self, joint_name, target, tolerance=0.5, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if joint_name == 'j0' and self.current_arm_pos is not None:
                if abs(self.current_arm_pos[0] - target) < tolerance:
                    self.get_logger().info(f"✓ J0 a atteint {target:.1f} mm")
                    return True
            elif joint_name == 'j1' and self.current_arm_pos is not None:
                if abs(self.current_arm_pos[1] - target) < tolerance:
                    self.get_logger().info(f"✓ J1 a atteint {target:.1f} deg")
                    return True
            elif joint_name == 'j2' and self.current_arm_pos is not None:
                if abs(self.current_arm_pos[2] - target) < tolerance:
                    self.get_logger().info(f"✓ J2 a atteint {target:.1f} mm")
                    return True
            elif joint_name == 'j3' and self.current_wrist_pos is not None:
                if abs(self.current_wrist_pos[0] - target) < tolerance:
                    self.get_logger().info(f"✓ J3 a atteint {target:.1f} mm")
                    return True
            elif joint_name == 'j4' and self.current_wrist_pos is not None:
                if abs(self.current_wrist_pos[1] - target) < tolerance:
                    self.get_logger().info(f"✓ J4 a atteint {target:.1f} mm")
                    return True
            elif joint_name == 'j5' and self.current_wrist_pos is not None:
                if abs(self.current_wrist_pos[2] - target) < tolerance:
                    self.get_logger().info(f"✓ J5 a atteint {target:.1f} deg")
                    return True
            
            try:
                if hasattr(self, 'cap') and self.cap is not None:
                    ret, frame = self.cap.read()
                    if ret:
                        disp = frame.copy()
                        cv2.putText(disp, f"ATTENTE: {joint_name} -> {target:.1f}", 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        pos_actuelle = self.current_arm_pos[0] if joint_name in ['j0','j1','j2'] else self.current_wrist_pos[0]
                        cv2.putText(disp, f"Position actuelle: {pos_actuelle:.1f}", 
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(disp, "q: Quitter", (10, disp.shape[0] - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.imshow("Detection des barres", disp)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            self.get_logger().info("Quitte pendant l'attente")
                            return False
            except Exception as e:
                pass
            
            time.sleep(0.01)
        
        self.get_logger().warn(f" Timeout: {joint_name} n'a pas atteint {target}")
        return False
    
    def move_j0_and_wait(self, target, tolerance=0.5, timeout=10.0):
        if self.current_arm_pos is None:
            self.get_logger().warn("Position J0 inconnue")
            return False
        if not (self.arm_limits['j0'][0] <= target <= self.arm_limits['j0'][1]):
            self.get_logger().warn(f"J0 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [target, self.current_arm_pos[1], self.current_arm_pos[2]]
        self.arm_pub.publish(msg)
        self.get_logger().info(f"J0: {self.current_arm_pos[0]:.1f} -> {target:.1f} mm")
        return self.wait_for_joint('j0', target, tolerance, timeout)
    
    def move_j1_and_wait(self, target, tolerance=0.5, timeout=10.0):
        if self.current_arm_pos is None:
            self.get_logger().warn("Position J1 inconnue")
            return False
        if not (self.arm_limits['j1'][0] <= target <= self.arm_limits['j1'][1]):
            self.get_logger().warn(f"J1 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [self.current_arm_pos[0], target, self.current_arm_pos[2]]
        self.arm_pub.publish(msg)
        self.get_logger().info(f"J1: {self.current_arm_pos[1]:.1f} -> {target:.1f} deg")
        return self.wait_for_joint('j1', target, tolerance, timeout)
    
    def move_j2_and_wait(self, target, tolerance=0.5, timeout=25.0):
        if self.current_arm_pos is None:
            self.get_logger().warn("Position J2 inconnue")
            return False
        if not (self.arm_limits['j2'][0] <= target <= self.arm_limits['j2'][1]):
            self.get_logger().warn(f"J2 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [self.current_arm_pos[0], self.current_arm_pos[1], target]
        self.arm_pub.publish(msg)
        self.get_logger().info(f"J2: {self.current_arm_pos[2]:.1f} -> {target:.1f} mm")
        return self.wait_for_joint('j2', target, tolerance, timeout)
    
    def move_j3_and_wait(self, target, tolerance=0.5, timeout=10.0):
        if self.current_wrist_pos is None:
            self.get_logger().warn("Position J3 inconnue")
            return False
        if not (self.wrist_limits['j3'][0] <= target <= self.wrist_limits['j3'][1]):
            self.get_logger().warn(f"J3 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [target, self.current_wrist_pos[1], self.current_wrist_pos[2]]
        self.wrist_pub.publish(msg)
        self.get_logger().info(f"J3: {self.current_wrist_pos[0]:.1f} -> {target:.1f} mm")
        return self.wait_for_joint('j3', target, tolerance, timeout)
    
    def move_j4_and_wait(self, target, tolerance=0.5, timeout=10.0):
        if self.current_wrist_pos is None:
            self.get_logger().warn("Position J4 inconnue")
            return False
        if not (self.wrist_limits['j4'][0] <= target <= self.wrist_limits['j4'][1]):
            self.get_logger().warn(f"J4 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [self.current_wrist_pos[0], target, self.current_wrist_pos[2]]
        self.wrist_pub.publish(msg)
        self.get_logger().info(f"J4: {self.current_wrist_pos[1]:.1f} -> {target:.1f} mm")
        return self.wait_for_joint('j4', target, tolerance, timeout)
    
    def move_j5_and_wait(self, target, tolerance=0.5, timeout=10.0):
        if self.current_wrist_pos is None:
            self.get_logger().warn("Position J5 inconnue")
            return False
        if not (self.wrist_limits['j5'][0] <= target <= self.wrist_limits['j5'][1]):
            self.get_logger().warn(f"J5 hors limite: {target}")
            return False
        msg = Float64MultiArray()
        msg.data = [self.current_wrist_pos[0], self.current_wrist_pos[1], target]
        self.wrist_pub.publish(msg)
        self.get_logger().info(f"J5: {self.current_wrist_pos[2]:.1f} -> {target:.1f} deg")
        return self.wait_for_joint('j5', target, tolerance, timeout)
    
    # ============================================================
    # Mouvements de base
    # ============================================================
    def move_j0(self, delta_mm):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[0] += delta_mm
            if self.arm_limits['j0'][0] <= new[0] <= self.arm_limits['j0'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                self.get_logger().info(f"MOVE J0: {self.current_arm_pos[0]:.1f} -> {new[0]:.1f} (delta={delta_mm:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J0 hors limite: {new[0]:.1f}")
                return False
        return False
    
    def move_j1(self, delta_deg):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[1] += delta_deg
            if self.arm_limits['j1'][0] <= new[1] <= self.arm_limits['j1'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                self.get_logger().info(f"MOVE J1: {self.current_arm_pos[1]:.1f} -> {new[1]:.1f} (delta={delta_deg:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J1 hors limite: {new[1]:.1f}")
                return False
        return False
    
    def move_j2(self, delta_mm):
        if self.current_arm_pos is not None:
            new = self.current_arm_pos.copy()
            new[2] += delta_mm
            if self.arm_limits['j2'][0] <= new[2] <= self.arm_limits['j2'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.arm_pub.publish(msg)
                self.get_logger().info(f"MOVE J2: {self.current_arm_pos[2]:.1f} -> {new[2]:.1f} (delta={delta_mm:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J2 hors limite: {new[2]:.1f}")
                return False
        return False
    
    def move_j3(self, delta_mm):
        if self.current_wrist_pos is not None:
            new = self.current_wrist_pos.copy()
            new[0] += delta_mm
            if self.wrist_limits['j3'][0] <= new[0] <= self.wrist_limits['j3'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.wrist_pub.publish(msg)
                self.get_logger().info(f"MOVE J3: {self.current_wrist_pos[0]:.1f} -> {new[0]:.1f} (delta={delta_mm:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J3 hors limite: {new[0]:.1f}")
                return False
        return False
    
    def move_j4(self, delta_mm):
        if self.current_wrist_pos is not None:
            new = self.current_wrist_pos.copy()
            new[1] += delta_mm
            if self.wrist_limits['j4'][0] <= new[1] <= self.wrist_limits['j4'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.wrist_pub.publish(msg)
                self.get_logger().info(f"MOVE J4: {self.current_wrist_pos[1]:.1f} -> {new[1]:.1f} (delta={delta_mm:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J4 hors limite: {new[1]:.1f}")
                return False
        return False
    
    def move_j5(self, delta_deg):
        if self.current_wrist_pos is not None:
            new = self.current_wrist_pos.copy()
            new[2] += delta_deg
            if self.wrist_limits['j5'][0] <= new[2] <= self.wrist_limits['j5'][1]:
                msg = Float64MultiArray()
                msg.data = [new[0], new[1], new[2]]
                self.wrist_pub.publish(msg)
                self.get_logger().info(f"MOVE J5: {self.current_wrist_pos[2]:.1f} -> {new[2]:.1f} (delta={delta_deg:+.1f})")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                return True
            else:
                self.get_logger().warn(f"J5 hors limite: {new[2]:.1f}")
                return False
        return False
    
    def wait_for_positions(self, timeout=3.0):
        start = time.time()
        while not self.positions_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.positions_received:
            self.get_logger().warn("Position actuelle inconnue, utilisation de 0")
            self.current_arm_pos = [0.0, 0.0, 0.0]
            self.current_wrist_pos = [0.0, 0.0, 0.0]
            self.positions_received = True
    
    def move_j0_to_target(self, target_mm, tolerance=2.0):
        self.wait_for_positions()
        if self.current_arm_pos is None:
            return False
        current = self.current_arm_pos[0]
        delta = target_mm - current
        if abs(delta) < tolerance:
            self.get_logger().info(f"J0 deja a {target_mm:.1f} mm")
            return True
        self.get_logger().info(f"J0: {current:.1f} -> {target_mm:.1f} mm")
        msg = Float64MultiArray()
        msg.data = [target_mm, self.current_arm_pos[1], self.current_arm_pos[2]]
        self.arm_pub.publish(msg)
        start = time.time()
        timeout = 15.0
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.01)
            if self.current_arm_pos is not None:
                if abs(self.current_arm_pos[0] - target_mm) < tolerance:
                    self.get_logger().info(f"✓ J0 atteint: {self.current_arm_pos[0]:.1f} mm")
                    return True
        self.get_logger().warn(f"J0: timeout, position: {self.current_arm_pos[0]:.1f} mm")
        return False
    
    # ============================================================
    # ALIGNEMENTS
    # ============================================================
    def align_vertical(self, frame, pts_cadre):
        self.wait_for_positions()
        if pts_cadre is None:
            self.get_logger().warn("Aucun cadre detecte pour l'alignement vertical")
            return frame, None
        
        h, w = frame.shape[:2]
        centre_cam_y = h // 2
        speed = 80
        dt = 0.1
        step = speed * dt
        deadband = 6.0
        max_iter = 300
        
        self.get_logger().info(f"Alignement vertical (J0): Step={step:.2f} mm, Deadband={deadband} px")
        result = frame.copy()
        
        for i in range(max_iter):
            ret, frame = self.cap.read()
            if not ret:
                break
            
            mask_cadre, _, _, _, _ = masque_cadre(frame)
            pts_cadre_detecte, _ = plus_grand_quadrilatere(mask_cadre)
            result = frame.copy()
            
            cv2.circle(result, (w//2, centre_cam_y), 8, (0, 165, 255), -1)
            cv2.line(result, (0, centre_cam_y), (w, centre_cam_y), (0, 0, 255), 2)
            cv2.putText(result, "CAM CENTER Y", (w//2 + 15, centre_cam_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            
            if pts_cadre_detecte is not None:
                cv2.polylines(result, [pts_cadre_detecte], True, (0, 255, 0), 2)
                centre_cadre = np.mean(pts_cadre_detecte, axis=0).astype(int)
                cv2.circle(result, tuple(centre_cadre), 8, (255, 0, 255), -1)
                cv2.line(result, (0, centre_cadre[1]), (w, centre_cadre[1]), (255, 0, 0), 2)
                cv2.putText(result, "CADRE CENTER Y", (centre_cadre[0] + 15, centre_cadre[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                
                diff_y = centre_cadre[1] - centre_cam_y
                couleur = (0, 255, 255) if diff_y > 0 else (255, 255, 0)
                cv2.putText(result, f"Diff Y: {diff_y:+d} px", (w-200, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 1)
                cv2.putText(result, f"Align V: {i+1}/{max_iter}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"J0: {self.current_arm_pos[0]:.1f} mm", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                if abs(diff_y) < deadband:
                    self.get_logger().info(f"Alignement vertical atteint (err={diff_y:+.1f} px)")
                    break
                if diff_y > 0:
                    self.move_j0(-step)
                else:
                    self.move_j0(step)
            else:
                cv2.putText(result, "Aucun cadre detecte", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                break
            
            cv2.putText(result, "q: quitter  ESC: annuler", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Detection des barres", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("Alignement vertical interrompu")
                break
            
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().warn("Alignement vertical : nombre max d'iterations atteint.")
        
        return result, pts_cadre_detecte
    
    def align_angle(self, frame, pts_cadre):
        self.wait_for_positions()
        if pts_cadre is None:
            self.get_logger().warn("Aucun cadre detecte pour l'alignement angulaire")
            return frame, None
        
        h, w = frame.shape[:2]
        centre_cam_y = h // 2
        dt = 0.1
        speed_angle = 40.0
        step_deg = speed_angle * dt
        deadband_angle = 0.5
        max_iter = 300
        
        self.get_logger().info(f"Alignement angulaire (J1): Step={step_deg:.2f} deg, Deadband={deadband_angle} deg")
        result = frame.copy()
        
        for i in range(max_iter):
            ret, frame = self.cap.read()
            if not ret:
                break
            
            mask_cadre, _, _, _, _ = masque_cadre(frame)
            pts_cadre_detecte, _ = plus_grand_quadrilatere(mask_cadre)
            result = frame.copy()
            
            cv2.circle(result, (w//2, centre_cam_y), 8, (0, 165, 255), -1)
            cv2.line(result, (0, centre_cam_y), (w, centre_cam_y), (0, 0, 255), 2)
            cv2.putText(result, "CAM CENTER", (w//2 + 15, centre_cam_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            
            if pts_cadre_detecte is not None:
                cv2.polylines(result, [pts_cadre_detecte], True, (0, 255, 0), 2)
                
                hg = pts_cadre_detecte[0]
                hd = pts_cadre_detecte[1]
                dx = hd[0] - hg[0]
                dy = hd[1] - hg[1]
                angle_deg = math.degrees(math.atan2(dy, dx))
                
                cv2.putText(result, f"Angle: {angle_deg:+.1f} deg", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.line(result, tuple(hg), tuple(hd), (0, 255, 255), 2)
                cv2.putText(result, f"Align Ang: {i+1}/{max_iter}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"J1: {self.current_arm_pos[1]:.1f} deg", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                if abs(angle_deg) < deadband_angle:
                    self.get_logger().info(f"Alignement angulaire atteint (angle={angle_deg:+.1f} deg)")
                    break
                if angle_deg > 0:
                    self.move_j1(-step_deg)
                else:
                    self.move_j1(step_deg)
            else:
                cv2.putText(result, "Aucun cadre detecte", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                break
            
            cv2.putText(result, "q: quitter  ESC: annuler", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Detection des barres", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("Alignement angulaire interrompu")
                break
            
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().warn("Alignement angulaire : nombre max d'iterations atteint.")
        
        return result, pts_cadre_detecte
    
    def align_vertical_centre(self, frame, pts_cadre):
        self.wait_for_positions()
        if pts_cadre is None:
            self.get_logger().warn("Aucun cadre detecte pour l'alignement vertical centre")
            return frame, None
        
        h, w = frame.shape[:2]
        centre_cam_x = w // 2
        speed = 60
        dt = 0.1
        step = speed * dt
        deadband = 6.0
        max_iter = 300
        
        self.get_logger().info(f"Alignement vertical centre (J3): Step={step:.2f} mm, Deadband={deadband} px")
        result = frame.copy()
        
        for i in range(max_iter):
            ret, frame = self.cap.read()
            if not ret:
                break
            
            mask_cadre, _, _, _, _ = masque_cadre(frame)
            pts_cadre_detecte, _ = plus_grand_quadrilatere(mask_cadre)
            result = frame.copy()
            
            cv2.circle(result, (centre_cam_x, h//2), 8, (0, 165, 255), -1)
            cv2.line(result, (centre_cam_x, 0), (centre_cam_x, h), (0, 0, 255), 2)
            cv2.putText(result, "CAM CENTER X", (centre_cam_x + 15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
            
            if pts_cadre_detecte is not None:
                cv2.polylines(result, [pts_cadre_detecte], True, (0, 255, 0), 2)
                centre_cadre = np.mean(pts_cadre_detecte, axis=0).astype(int)
                cv2.circle(result, tuple(centre_cadre), 8, (255, 0, 255), -1)
                cv2.line(result, (centre_cadre[0], 0), (centre_cadre[0], h), (255, 0, 0), 2)
                cv2.putText(result, "CADRE CENTER X", (centre_cadre[0] + 15, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                
                diff_x = centre_cadre[0] - centre_cam_x
                couleur = (0, 255, 255) if diff_x > 0 else (255, 255, 0)
                cv2.putText(result, f"Diff X: {diff_x:+d} px", (w-200, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 1)
                cv2.putText(result, f"Align Center: {i+1}/{max_iter}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(result, f"J3: {self.current_wrist_pos[0]:.1f} mm", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                if abs(diff_x) < deadband:
                    self.get_logger().info(f"Alignement vertical centre atteint (err={diff_x:+.1f} px)")
                    break
                if diff_x > 0:
                    self.move_j3(step)
                else:
                    self.move_j3(-step)
            else:
                cv2.putText(result, "Aucun cadre detecte", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                break
            
            cv2.putText(result, "q: quitter  ESC: annuler", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Detection des barres", result)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("Alignement vertical centre interrompu")
                break
            
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().warn("Alignement vertical centre : nombre max d'iterations atteint.")
        
        return result, pts_cadre_detecte
    
    def aligner_complet(self, frame, pts_cadre, max_iterations=10):
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("ALIGNEMENTS ITERATIFS J1 <-> J0")
        self.get_logger().info("=" * 40)
        
        tolerance_j0 = 6.0
        tolerance_j1 = 0.5
        
        for iteration in range(max_iterations):
            self.get_logger().info(f"\n--- Iteration {iteration + 1}/{max_iterations} ---")
            
            self.get_logger().info("→ Alignement angulaire (J1)...")
            _, pts_cadre = self.align_angle(frame, pts_cadre)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
            
            if pts_cadre is None:
                self.get_logger().warn(" Cadre perdu pendant l'alignement J1")
                return None
            
            self.get_logger().info("→ Alignement vertical (J0)...")
            _, pts_cadre = self.align_vertical(frame, pts_cadre)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
            
            if pts_cadre is None:
                self.get_logger().warn(" Cadre perdu pendant l'alignement J0")
                return None
            
            h, w = frame.shape[:2]
            centre_cam_y = h // 2
            centre_cadre = np.mean(pts_cadre, axis=0).astype(int)
            diff_y = abs(centre_cadre[1] - centre_cam_y)
            
            hg = pts_cadre[0]
            hd = pts_cadre[1]
            dx = hd[0] - hg[0]
            dy = hd[1] - hg[1]
            angle_deg = math.degrees(math.atan2(dy, dx))
            
            self.get_logger().info(f"  → Erreur J0: {diff_y:.1f} px, Erreur J1: {angle_deg:.2f}°")
            
            if diff_y < tolerance_j0 and abs(angle_deg) < tolerance_j1:
                self.get_logger().info(" Alignements convergés !")
                return pts_cadre
            
            if iteration == max_iterations - 1:
                self.get_logger().warn(f" Alignements non convergés après {max_iterations} itérations")
                self.get_logger().warn(f"   Erreur J0: {diff_y:.1f} px (tolerance: {tolerance_j0})")
                self.get_logger().warn(f"   Erreur J1: {angle_deg:.2f}° (tolerance: {tolerance_j1})")
        
        return pts_cadre
    
    # ============================================================
    # DESCENTE CALCULEE
    # ============================================================
    def descente_calculee(self):
        self.get_logger().info("=== PHASE DE DESCENTE (sans caméra) ===")
        
        if self.z_cam_actuel is None:
            self.get_logger().warn("Aucune valeur Z disponible")
            return
        
        z_actuel_mm = self.z_cam_actuel
        z_cible_mm = 50.0 + self.OFFSET_HAUTEUR_PINCE
        
        if z_actuel_mm <= z_cible_mm:
            self.get_logger().warn(f"Z_actuel ({z_actuel_mm:.1f} mm) est deja <= {z_cible_mm} mm")
            return
        
        distance_totale_mm = z_actuel_mm - z_cible_mm
        
        distance_j2_mm = distance_totale_mm * (2.0 / 3.0)
        distance_j4_mm = distance_totale_mm * (1.0 / 3.0)
        compensation_j0_mm = distance_totale_mm * self.SIN_ALPHA_X
        compensation_j3_mm = distance_totale_mm * self.SIN_ALPHA_Y
        
        self.get_logger().info(f"--- CALCULS (basés sur Z={z_actuel_mm:.1f} mm) ---")
        self.get_logger().info(f"Distance totale: {distance_totale_mm:.1f} mm")
        self.get_logger().info(f"  J2: -{distance_j2_mm:.1f} mm (2/3)")
        self.get_logger().info(f"  J4: -{distance_j4_mm:.1f} mm (1/3)")
        self.get_logger().info(f" J0 (horizontal): +{compensation_j0_mm:.1f} mm")
        self.get_logger().info(f"Compensation J3 (vertical): +{compensation_j3_mm:.1f} mm")
        self.get_logger().info("--- DEBUT DES MOUVEMENTS ---")
        
        if self.current_arm_pos is None or self.current_wrist_pos is None:
            self.get_logger().warn("Positions inconnues, impossible de descendre")
            return
        
        target_j2 = self.current_arm_pos[2] - distance_j2_mm
        target_j4 = self.current_wrist_pos[1] - distance_j4_mm
        target_j0 = self.current_arm_pos[0] + compensation_j0_mm
        target_j3 = self.current_wrist_pos[0] + compensation_j3_mm
        
        if distance_j2_mm > 0:
            self.get_logger().info(f"1. Descente J2: {self.current_arm_pos[2]:.1f} -> {target_j2:.1f} mm")
            self.move_j2_and_wait(target_j2)
        
        if distance_j4_mm > 0:
            self.get_logger().info(f"2. Descente J4: {self.current_wrist_pos[1]:.1f} -> {target_j4:.1f} mm")
            self.move_j4_and_wait(target_j4)
        
        if compensation_j0_mm != 0:
            self.get_logger().info(f"3. Compensation J0: {self.current_arm_pos[0]:.1f} -> {target_j0:.1f} mm")
            self.move_j0_and_wait(target_j0)
        
        if compensation_j3_mm != 0:
            self.get_logger().info(f"4. Compensation J3: {self.current_wrist_pos[0]:.1f} -> {target_j3:.1f} mm")
            self.move_j3_and_wait(target_j3)
        
        if self.current_arm_pos is not None:
            self.j0_centre = self.current_arm_pos[0]
            self.get_logger().info(f" J0 centre mis à jour après descente : {self.j0_centre:.1f} mm")
        
        self.get_logger().info("=== DESCENTE TERMINEE ===")
        self.get_logger().info(f"Le robot est theoriquement a {z_cible_mm:.1f} mm du cadre")
    
    # ============================================================
    # INSPECTION D'UNE BARRE (MODIFIÉE AVEC VÉRIFICATION)
    # ============================================================
    def inspect_barre(self, barre_num):
        """
        Inspection d'une barre avec vérification de Z.
        """
        self.get_logger().info(f"=== INSPECTION DE LA BARRE {barre_num} ===")
        
        # VÉRIFICATION CRITIQUE : Z doit être mesuré
        if self.z_cam_actuel is None:
            self.get_logger().error(f" Z non mesuré ! Impossible d'inspecter la barre {barre_num}")
            self.get_logger().error("   → Faites d'abord un alignement (A) ou mesurez Z (Z)")
            self.get_logger().error("   → Barre ignorée")
            return
        
        if self.current_arm_pos is None or self.current_wrist_pos is None:
            self.get_logger().warn("Positions inconnues, inspection impossible")
            return
        
        # Sauvegarder la position J4 actuelle
        j4_initial = self.current_wrist_pos[1]
        
        # Sauvegarder la hauteur J2 initiale
        j2_initial = self.current_arm_pos[2]
        
        # ÉTAPE 0 : Vérifier que J5 est à 0
        j5_actuel = self.current_wrist_pos[2]
        if abs(j5_actuel) > self.TOLERANCE_J5_ZERO:
            self.get_logger().warn(f"J5 n'est pas à 0 ! Actuel: {j5_actuel:.1f}°")
            self.get_logger().info(f"→ Remise de J5 à 0 avant la descente...")
            self.move_j5_and_wait(0.0)
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            self.get_logger().info(f" J5 est bien à 0 ({j5_actuel:.1f}°)")
        
        # ÉTAPE 1 : Remonter J2
        target_j2_haut = j2_initial + self.HAUTEUR_REMONTEE_INSPECTION
        self.get_logger().info(f"1. Remontee J2 de {self.HAUTEUR_REMONTEE_INSPECTION:.1f} mm: {j2_initial:.1f} -> {target_j2_haut:.1f} mm")
        self.move_j2_and_wait(target_j2_haut)
        
        # ÉTAPE 2 : Tourner J5 à +90°
        self.get_logger().info(f"2. Tourner J5 a +{self.ANGLE_J5_POSITIF}°: {self.current_wrist_pos[2]:.1f} -> {self.ANGLE_J5_POSITIF:.1f} deg")
        self.move_j5_and_wait(self.ANGLE_J5_POSITIF)
        
        # ÉTAPE 3 : Attendre
        self.get_logger().info(f"3. Attente {self.TEMPS_ATTENTE_INSPECTION} secondes...")
        time.sleep(self.TEMPS_ATTENTE_INSPECTION)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # ÉTAPE 4 : Tourner J5 à -90°
        self.get_logger().info(f"4. Tourner J5 a {self.ANGLE_J5_NEGATIF}°: {self.current_wrist_pos[2]:.1f} -> {self.ANGLE_J5_NEGATIF:.1f} deg")
        self.move_j5_and_wait(self.ANGLE_J5_NEGATIF)
        
        # ÉTAPE 5 : Attendre
        self.get_logger().info(f"5. Attente {self.TEMPS_ATTENTE_INSPECTION} secondes...")
        time.sleep(self.TEMPS_ATTENTE_INSPECTION)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # ÉTAPE 6 : Remettre J5 à 0
        self.get_logger().info(f"6. Remettre J5 a 0°: {self.current_wrist_pos[2]:.1f} -> 0.0 deg")
        self.move_j5_and_wait(0.0)
        
        if abs(self.current_wrist_pos[2]) > self.TOLERANCE_J5_ZERO:
            self.get_logger().error(f" J5 n'est pas revenu à 0 ! Actuel: {self.current_wrist_pos[2]:.1f}°")
            self.get_logger().error("   → Réessai de remise à 0...")
            self.move_j5_and_wait(0.0)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.get_logger().info(f" J5 confirmé à 0°")
        
        # ÉTAPE 7 : Redescendre J2
        self.get_logger().info(f"7. Redescente J2 de {self.HAUTEUR_REMONTEE_INSPECTION:.1f} mm: {self.current_arm_pos[2]:.1f} -> {j2_initial:.1f} mm")
        self.move_j2_and_wait(j2_initial)
        
        # Vérifications finales
        if abs(self.current_wrist_pos[1] - j4_initial) > self.TOLERANCE_J4:
            self.get_logger().warn(f" J4 a bougé de {self.current_wrist_pos[1] - j4_initial:.1f} mm! Remise à {j4_initial:.1f}")
            self.move_j4_and_wait(j4_initial)
        
        if abs(self.current_wrist_pos[2]) > self.TOLERANCE_J5_ZERO:
            self.get_logger().error(f"J5 n'est plus à 0 après descente ! Actuel: {self.current_wrist_pos[2]:.1f}°")
        
        if abs(self.current_arm_pos[2] - j2_initial) > self.TOLERANCE_J2:
            self.get_logger().warn(f" J2 n'est pas revenu à sa position initiale ! Actuel: {self.current_arm_pos[2]:.1f} mm, Attendu: {j2_initial:.1f} mm")
        
        self.get_logger().info(f"=== FIN INSPECTION BARRE {barre_num} ===")
        self.get_logger().info(f"   J5 = {self.current_wrist_pos[2]:.1f}° (doit être 0)")
        self.get_logger().info(f"   J2 = {self.current_arm_pos[2]:.1f} mm (doit être {j2_initial:.1f} mm)")
    
    # ============================================================
    # NAVIGATION VERS LES BARRES (MODIFIÉE AVEC VÉRIFICATION)
    # ============================================================
    def navigation_vers_barres(self):
        """
        Navigue vers les barres et effectue l'inspection avec vérifications.
        """
        #  VÉRIFICATION : Z doit être mesuré
        if self.z_cam_actuel is None:
            self.get_logger().error(" Z non mesuré ! Impossible de naviguer vers les barres")
            self.get_logger().error("   → Faites d'abord un alignement (A) ou mesurez Z (Z)")
            self.get_logger().error("   → Navigation annulée")
            return
        
        #  VÉRIFICATION : J2 doit être descendu
        if self.current_arm_pos is not None and self.current_arm_pos[2] > -10.0:
            self.get_logger().warn(f" J2 n'est pas descendu ! (position actuelle: {self.current_arm_pos[2]:.1f} mm)")
            self.get_logger().info("   → Descente automatique...")
            self.descente_calculee()
            time.sleep(2)
            rclpy.spin_once(self, timeout_sec=0.0)
            
            if self.current_arm_pos is not None and self.current_arm_pos[2] > -10.0:
                self.get_logger().error(" Descente automatique échouée")
                self.get_logger().error("   → Navigation annulée")
                return
        
        self.get_logger().info(f"   Navigation vers {len(self.liste_barres)} barres...")
        
        for i, barre in enumerate(self.liste_barres, 1):
            self.get_logger().info(f"  🔹 Barre {barre} ({i}/{len(self.liste_barres)})")
            
            j0_target = self.j0_centre + self.barres_offsets[barre] + self.OFFSET_PINCE
            self.get_logger().info(f"     J0 cible : {j0_target:.1f} mm")
            
            self.move_j0_and_wait(j0_target)
            time.sleep(0.5)
            
            self.inspect_barre(barre)
            time.sleep(0.5)
    
    # ============================================================
    # SEQUENCE COMPLETE
    # ============================================================
    def sequence_complete(self):
        self.wait_for_positions()
        self.get_logger().info("=" * 60)
        self.get_logger().info("=== DEBUT DE LA SEQUENCE COMPLETE ===")
        self.get_logger().info("=" * 60)
        
        # PHASE 1: ALIGNEMENTS
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("PHASE 1: ALIGNEMENTS ITERATIFS")
        self.get_logger().info("=" * 40)
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Impossible de lire la camera")
            return
        
        mask_cadre, _, _, _, _ = masque_cadre(frame)
        pts_cadre, _ = plus_grand_quadrilatere(mask_cadre)
        
        if pts_cadre is None:
            self.get_logger().error("Aucun cadre detecte !")
            return
        
        pts_cadre = self.aligner_complet(frame, pts_cadre, max_iterations=10)
        
        if pts_cadre is None:
            self.get_logger().error("Impossible d'aligner le cadre")
            return
        
        self.get_logger().info("\n→ Alignement vertical centre (J3)...")
        _, pts_cadre = self.align_vertical_centre(frame, pts_cadre)
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # Réinitialiser J5
        self.get_logger().info("\n→ Réinitialisation de J5 à 0°...")
        if self.current_wrist_pos is not None:
            j5_actuel = self.current_wrist_pos[2]
            if abs(j5_actuel) > self.TOLERANCE_J5_ZERO:
                delta_j5 = 0.0 - j5_actuel
                self.get_logger().info(f"  J5: {j5_actuel:.1f}° -> 0.0°")
                self.move_j5(delta_j5)
                time.sleep(0.5)
                rclpy.spin_once(self, timeout_sec=0.0)
        
        # Enregistrer J0 centre
        if self.current_arm_pos is not None:
            self.j0_centre = self.current_arm_pos[0]
            self.get_logger().info(f" J0 centre enregistre : {self.j0_centre:.1f} mm")
        
        #  Mesurer Z depuis le cadre (même sans barres)
        self.get_logger().info("\n→ Mesure de Z depuis le cadre...")
        z_mm = self.calculer_z_depuis_cadre(pts_cadre, frame.shape)
        if z_mm is not None:
            self.z_cam_actuel = z_mm
            self.get_logger().info(f" Z mesuré depuis le cadre : {self.z_cam_actuel:.1f} mm")
        else:
            self.get_logger().warn(" Impossible de mesurer Z depuis le cadre")
            self.get_logger().warn(" Les barres ne pourront pas être inspectées")
        
        self.get_logger().info("\n ALIGNEMENTS TERMINES")
        
        # PHASE 2: DESCENTE
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("PHASE 2: DESCENTE A 2 CM")
        self.get_logger().info("=" * 40)
        
        self.descente_calculee()
        
        # PHASE 3: NAVIGATION + INSPECTION
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("PHASE 3: NAVIGATION + INSPECTION")
        self.get_logger().info("=" * 40)
        self.get_logger().info(f"Barres a visiter : {self.liste_barres}")
        
        offset_initial = self.OFFSET_PINCE
        rotation_effectuee = False
        barres_ignorees = []
        
        for i, barre in enumerate(self.liste_barres):
            if barre < 1 or barre > 10:
                self.get_logger().warn(f"Barre {barre} invalide (1 a 10)")
                continue
            
            j0_target = self.j0_centre + self.barres_offsets[barre] + self.OFFSET_PINCE
            
            self.get_logger().info(f"\n{'='*30}")
            self.get_logger().info(f"BARRE {barre} ({i+1}/{len(self.liste_barres)})")
            self.get_logger().info(f"J0 actuel: {self.current_arm_pos[0]:.1f} mm")
            self.get_logger().info(f"J0 centre: {self.j0_centre:.1f} mm")
            self.get_logger().info(f"Offset barre: {self.barres_offsets[barre]:.1f} mm")
            self.get_logger().info(f"Offset pince: +{self.OFFSET_PINCE:.1f} mm")
            self.get_logger().info(f"J0 cible : {j0_target:.1f} mm")
            
            if j0_target < self.arm_limits['j0'][0] or j0_target > self.arm_limits['j0'][1]:
                self.get_logger().warn(f" Barre {barre} hors limite: {j0_target:.1f} mm")
                
                if not rotation_effectuee:
                    self.get_logger().info("→ Rotation J5 à -150° pour réduire l'offset...")
                    if self.current_wrist_pos is not None:
                        target_j2 = self.current_arm_pos[2] + self.HAUTEUR_REMONTEE_ROTATION
                        self.get_logger().info(f"  Remontée J2 de {self.HAUTEUR_REMONTEE_ROTATION:.1f} mm")
                        self.move_j2_and_wait(target_j2)
                        time.sleep(0.5)
                        
                        target_j5 = self.ANGLE_J5_ROTATION
                        delta_j5 = target_j5 - self.current_wrist_pos[2]
                        self.get_logger().info(f"  J5: {self.current_wrist_pos[2]:.1f}° -> {target_j5:.1f}°")
                        self.move_j5(delta_j5)
                        time.sleep(2.0)
                        
                        if self.current_wrist_pos is not None:
                            j5_actuel = self.current_wrist_pos[2]
                            if abs(j5_actuel - target_j5) <= self.TOLERANCE_J5:
                                self.OFFSET_PINCE = 0.0
                            else:
                                angle_effectif = abs(j5_actuel)
                                if angle_effectif >= 140.0:
                                    self.OFFSET_PINCE = 0.0
                                elif angle_effectif >= 100.0:
                                    self.OFFSET_PINCE = 130.0 * (1 - (angle_effectif / 150.0))
                                else:
                                    self.OFFSET_PINCE = 130.0
                                self.OFFSET_PINCE = max(0.0, min(130.0, self.OFFSET_PINCE))
                            rotation_effectuee = True
                            self.get_logger().info(f" OFFSET_PINCE = {self.OFFSET_PINCE:.1f} mm")
                        
                        target_j2 = self.current_arm_pos[2] - self.HAUTEUR_REMONTEE_ROTATION
                        self.move_j2_and_wait(target_j2)
                        time.sleep(0.5)
                        
                        j0_target = self.j0_centre + self.barres_offsets[barre] + self.OFFSET_PINCE
                        if j0_target < self.arm_limits['j0'][0] or j0_target > self.arm_limits['j0'][1]:
                            self.get_logger().warn(f" Barre {barre} toujours hors limite")
                            barres_ignorees.append(barre)
                            continue
                else:
                    self.get_logger().warn(f" Barre {barre} toujours hors limite après rotation J5")
                    barres_ignorees.append(barre)
                    continue
            
            self.get_logger().info(f"{'='*30}")
            self.get_logger().info(f"→ Navigation vers la barre {barre}...")
            self.move_j0_and_wait(j0_target)
            time.sleep(0.5)
            
            self.inspect_barre(barre)
            time.sleep(0.5)
        
        if barres_ignorees:
            self.get_logger().warn(f"\n Barres ignorées (hors limite): {barres_ignorees}")
        else:
            self.get_logger().info("\n Toutes les barres ont été visitées")
        
        self.get_logger().info("\n NAVIGATION + INSPECTION TERMINEES")
        
        # PHASE 4: RETOUR HOME
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("PHASE 4: RETOUR HOME")
        self.get_logger().info("=" * 40)
        
        positions_finales, tous_a_zero = self.go_home()
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("=== VÉRIFICATION FINALE DE LA SÉQUENCE ===")
        self.get_logger().info("=" * 60)
        
        if tous_a_zero:
            self.get_logger().info(" Tous les joints sont à 0 - Séquence réussie !")
        else:
            self.get_logger().warn(" Des joints ne sont pas à 0 - Vérifier le retour home")
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("=== SEQUENCE COMPLETE TERMINEE ===")
        self.get_logger().info("=" * 60)
    
    # ============================================================
    # GO HOME
    # ============================================================
    def go_home(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("=== RETOUR HOME (TOUS LES JOINTS A 0) ===")
        self.get_logger().info("=" * 60)
        self.wait_for_positions()
        
        positions_finales = {
            'j0': None, 'j1': None, 'j2': None,
            'j3': None, 'j4': None, 'j5': None
        }
        erreurs = []
        
        if self.current_arm_pos is not None:
            self.get_logger().info("1. J2 -> 0")
            if not self.move_j2_and_wait(0.0):
                erreurs.append("J2 n'a pas atteint 0")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        if self.current_wrist_pos is not None:
            self.get_logger().info("2. J5 -> 0")
            if not self.move_j5_and_wait(0.0):
                erreurs.append("J5 n'a pas atteint 0")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.get_logger().info("3. J0 -> 0")
        if not self.move_j0_and_wait(0.0):
            erreurs.append("J0 n'a pas atteint 0")
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        if self.current_arm_pos is not None:
            self.get_logger().info("4. J1 -> 0")
            if not self.move_j1_and_wait(0.0):
                erreurs.append("J1 n'a pas atteint 0")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        if self.current_wrist_pos is not None:
            self.get_logger().info("5. J3 -> 0")
            if not self.move_j3_and_wait(0.0):
                erreurs.append("J3 n'a pas atteint 0")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        if self.current_wrist_pos is not None:
            self.get_logger().info("6. J4 -> 0")
            if not self.move_j4_and_wait(0.0):
                erreurs.append("J4 n'a pas atteint 0")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)
        
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.0)
        
        if self.current_arm_pos is not None and self.current_wrist_pos is not None:
            positions_finales['j0'] = self.current_arm_pos[0]
            positions_finales['j1'] = self.current_arm_pos[1]
            positions_finales['j2'] = self.current_arm_pos[2]
            positions_finales['j3'] = self.current_wrist_pos[0]
            positions_finales['j4'] = self.current_wrist_pos[1]
            positions_finales['j5'] = self.current_wrist_pos[2]
            
            self.get_logger().info("\n POSITIONS FINALES DES JOINTS :")
            self.get_logger().info("=" * 40)
            self.get_logger().info(f"  J0 : {positions_finales['j0']:8.1f} mm  {'✅' if abs(positions_finales['j0']) < 0.5 else '❌'}")
            self.get_logger().info(f"  J1 : {positions_finales['j1']:8.1f} deg {'✅' if abs(positions_finales['j1']) < 0.5 else '❌'}")
            self.get_logger().info(f"  J2 : {positions_finales['j2']:8.1f} mm  {'✅' if abs(positions_finales['j2']) < 0.5 else '❌'}")
            self.get_logger().info(f"  J3 : {positions_finales['j3']:8.1f} mm  {'✅' if abs(positions_finales['j3']) < 0.5 else '❌'}")
            self.get_logger().info(f"  J4 : {positions_finales['j4']:8.1f} mm  {'✅' if abs(positions_finales['j4']) < 0.5 else '❌'}")
            self.get_logger().info(f"  J5 : {positions_finales['j5']:8.1f} deg {'✅' if abs(positions_finales['j5']) < 0.5 else '❌'}")
            self.get_logger().info("=" * 40)
            
            tous_a_zero = all(abs(v) < 0.5 for v in positions_finales.values() if v is not None)
        else:
            self.get_logger().error(" Impossible de récupérer les positions finales !")
            tous_a_zero = False
        
        if erreurs:
            self.get_logger().warn("\n ERREURS DÉTECTÉES :")
            for err in erreurs:
                self.get_logger().warn(f"  - {err}")
        else:
            self.get_logger().info("\n Aucune erreur détectée lors du retour home")
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("===  RETOUR HOME TERMINE ===")
        self.get_logger().info("=" * 60)
        
        return positions_finales, tous_a_zero
    
    # ============================================================
    # BOUCLE PRINCIPALE
    # ============================================================
    def run(self):
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            print("Aucune camera trouvee")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("=" * 60)
        print("DETECTION DES BARRES + INSPECTION")
        print("=" * 60)
        print("Touches :")
        print("  'b' : Annuler le figage")
        print("  'd' : Debug")
        print("  'c' : Afficher les coordonnees 3D")
        print("  'a' : Alignement vertical (J0)")
        print("  'g' : Alignement angulaire (J1)")
        print("  'v' : Alignement vertical centre (J3)")
        print("  's' : Sequence complete + inspection des barres")
        print("  'f' : Charger un fichier de commandes")
        print("  'h' : Afficher l'aide")
        print("  'q' : Quitter")
        print("=" * 60)
        print("Barres a visiter :", self.liste_barres)
        print("=" * 60)

        img_vide_320 = image_vide(320, 240, "Pas de donnees")
        img_vide_warp = image_vide(WARP_LARGEUR, WARP_HAUTEUR, "Pas de cadre/ROI")
        img_vide_hist = image_vide(150, WARP_HAUTEUR, "Pas de donnees")

        fenetres_principales = ["Detection des barres"]
        fenetres_debug = [
            # "Debug - Masque Cadre",
            # "Debug - Image redressee",
            # "Debug - Masque BLANC (trous remplis)",
            # "Debug - Histogramme",
            # "Debug - Orange",
            # "Debug - Jaune (cadre)",
            "Debug - Blanc (cadre)",
            #"Debug - Beige"
        ]
        
        for nom in fenetres_principales + fenetres_debug:
            cv2.namedWindow(nom, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                break

            h, w = frame.shape[:2]

            mask_cadre, mask_orange, mask_jaune, mask_blanc_cadre, mask_beige = masque_cadre(frame)
            pts_cadre_detecte, _ = plus_grand_quadrilatere(mask_cadre)

            result = frame.copy()
            barres_image = []
            warp = None
            mask_warp = None
            ligne_ratio = None
            espaces = []
            barres_brutes = []
            coords_premiere_barre = None
            nb_barres_actuel = 0
            
            if self.roi_memorise and self.roi_points is not None:
                pts_actifs = self.roi_points
            else:
                pts_actifs = pts_cadre_detecte

            if pts_actifs is not None:
                M, M_inv = calculer_homographies(pts_actifs)
                warp = redresser_image(frame, M)
                
                mask_warp = masque_blanc_warp(warp)
                barres_brutes, ligne_ratio, espaces = detecter_barres_et_espaces(mask_warp)
                barres_image = barres_vers_image_originale(barres_brutes, M_inv)
                barres_image = sorted(barres_image, key=lambda b: b['y_warp'])

                barres_inversees = list(reversed(barres_image))
                nb_barres_total = len(barres_image)
                nb_barres_actuel = len(barres_inversees)
                
                if nb_barres_actuel == NB_BARRES_AUTO_MEMO:
                    if pts_cadre_detecte is not None:
                        nouveaux_pts = pts_cadre_detecte.copy()
                    else:
                        nouveaux_pts = pts_actifs.copy() if pts_actifs is not None else None
                    
                    if nouveaux_pts is not None:
                        self.roi_points = nouveaux_pts.copy()
                        self.roi_memorise = True
                        self.nb_barres_memo = nb_barres_actuel
                        
                        self.barres_figees = barres_image.copy()
                        self.pts_figes = nouveaux_pts.copy()
                        
                        if len(barres_image) > 0:
                            coords = calculer_coordonnees_barre(
                                barres_image[0], self.pts_figes, frame.shape, M_inv
                            )
                            self.coords_figees = coords
                            coords_premiere_barre = coords
                            self.z_cam_actuel = coords['z_mm']
                        
                        self.mise_a_jour_effectuee = True
                        self.en_attente = False
                        self.auto_memo_active = True
                        
                        if self.auto_memo_active:
                            self.roi_log_counter += 1
                            if self.roi_log_counter % self.roi_log_interval == 0:
                                print(f"ROI mise a jour: Z={self.z_cam_actuel:.1f} mm")
                            self.auto_memo_active = False
                else:
                    self.en_attente = True
                    self.mise_a_jour_effectuee = False
                    
                    if self.barres_figees is not None:
                        coords_premiere_barre = self.coords_figees if self.show_coords else None

            # Affichage
            if self.barres_figees is not None and self.pts_figes is not None:
                coords_affichees = self.coords_figees if self.show_coords else None
                result = dessiner_resultat(result, self.pts_figes, self.barres_figees, self.roi_memorise,
                                           coords_affichees, len(self.barres_figees), self.auto_memo_active,
                                           self.mise_a_jour_effectuee, self.derniere_mise_a_jour,
                                           nb_barres_actuel, self.nb_barres_memo, self.barres_figees, self.pts_figes,
                                           self.en_attente)
            else:
                result = dessiner_resultat(result, pts_cadre_detecte, barres_image, self.roi_memorise,
                                           coords_premiere_barre, 0, self.auto_memo_active,
                                           self.mise_a_jour_effectuee, self.derniere_mise_a_jour,
                                           nb_barres_actuel, self.nb_barres_memo, None, None, False)

            if pts_cadre_detecte is None and self.barres_figees is None:
                cv2.putText(result, "Aucun rectangle detecte", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Detection des barres", result)

            # Debug
            if self.debug_mode:
                # cv2.imshow("Debug - Masque Cadre", cv2.resize(mask_cadre, (320, 240)))
                # cv2.imshow("Debug - Orange", cv2.resize(mask_orange, (320, 240)))
                # cv2.imshow("Debug - Jaune (cadre)", cv2.resize(mask_jaune, (320, 240)))
                cv2.imshow("Debug - Blanc (cadre)", cv2.resize(mask_blanc_cadre, (320, 240)))
                # cv2.imshow("Debug - Beige", cv2.resize(mask_beige, (320, 240)))

                # if mask_warp is not None:
                #     cv2.imshow("Debug - Masque BLANC (trous remplis)", mask_warp)
                # else:
                #     cv2.imshow("Debug - Masque BLANC (trous remplis)", img_vide_warp)

                # if warp is not None:
                #     warp_disp = dessiner_warp_avec_barres(warp, barres_brutes, espaces)
                #     cv2.imshow("Debug - Image redressee", warp_disp)
                # else:
                #     cv2.imshow("Debug - Image redressee", img_vide_warp)

                # if ligne_ratio is not None:
                #     hist_img = dessiner_histogramme(ligne_ratio, WARP_HAUTEUR)
                #     cv2.imshow("Debug - Histogramme", hist_img)
                # else:
                #     cv2.imshow("Debug - Histogramme", img_vide_hist)
            else:
                for nom in fenetres_debug:
                    if "warp" in nom or "Histogramme" in nom:
                        continue
                    cv2.imshow(nom, img_vide_320)
                # cv2.imshow("Debug - Image redressee", img_vide_warp)
                # cv2.imshow("Debug - Masque BLANC (trous remplis)", img_vide_warp)
                # cv2.imshow("Debug - Histogramme", img_vide_hist)

            # Gestion des touches
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'):
                self.debug_mode = not self.debug_mode
                print(f"Debug: {'ON' if self.debug_mode else 'OFF'}")
            elif key == ord('c'):
                self.show_coords = not self.show_coords
                print(f"Affichage des coordonnees: {'ON' if self.show_coords else 'OFF'}")
            elif key == ord('b'):
                if self.roi_memorise and self.barres_figees is not None:
                    self.roi_points = None
                    self.roi_memorise = False
                    self.nb_barres_memo = 0
                    self.auto_memo_active = False
                    self.mise_a_jour_effectuee = False
                    self.en_attente = False
                    
                    self.barres_figees = None
                    self.pts_figes = None
                    self.coords_figees = None
                    self.z_cam_actuel = None
                    
                    print("Figage annule - retour en mode detection")
                else:
                    print("Aucun figage actif")
            elif key == ord('a'):
                print("Lancement de l'alignement vertical (J0)...")
                ret, frame_temp = self.cap.read()
                if ret:
                    mask_cadre_temp, _, _, _, _ = masque_cadre(frame_temp)
                    pts_cadre_temp, _ = plus_grand_quadrilatere(mask_cadre_temp)
                    if pts_cadre_temp is not None:
                        self.align_vertical(frame_temp, pts_cadre_temp)
                print("Alignement vertical termine")
            elif key == ord('g'):
                print("Lancement de l'alignement angulaire (J1)...")
                ret, frame_temp = self.cap.read()
                if ret:
                    mask_cadre_temp, _, _, _, _ = masque_cadre(frame_temp)
                    pts_cadre_temp, _ = plus_grand_quadrilatere(mask_cadre_temp)
                    if pts_cadre_temp is not None:
                        self.align_angle(frame_temp, pts_cadre_temp)
                print("Alignement angulaire termine")
            elif key == ord('v'):
                print("Lancement de l'alignement vertical centre (J3)...")
                ret, frame_temp = self.cap.read()
                if ret:
                    mask_cadre_temp, _, _, _, _ = masque_cadre(frame_temp)
                    pts_cadre_temp, _ = plus_grand_quadrilatere(mask_cadre_temp)
                    if pts_cadre_temp is not None:
                        self.align_vertical_centre(frame_temp, pts_cadre_temp)
                print("Alignement vertical centre termine")
            elif key == ord('s'):
                print("\n" + "=" * 60)
                print("Lancement de la sequence complete...")
                print("=" * 60)
                print("\nEntrez les numeros des barres a inspecter (ex: 4,1,2)")
                entree = input("Barres : ").strip()
                if entree != "":
                    try:
                        self.liste_barres = [int(x.strip()) for x in entree.split(",") if x.strip().isdigit()]
                        print(f" Barres selectionnees : {self.liste_barres}")
                    except:
                        print(" Entree invalide, utilisation de [1,2,3]")
                        self.liste_barres = [1, 2, 3]
                else:
                    self.liste_barres = [1, 2, 3]
                    print(f"Barres par defaut : {self.liste_barres}")
                
                self.sequence_complete()
                print("\n Sequence complete terminee")
            elif key == ord('f') or key == ord('F'):
                print("\n Chargement d'un fichier de commandes")
                chemin = input("Chemin du fichier : ").strip()
                if chemin:
                    self.executer_fichier_commandes(chemin)
                else:
                    print(" Aucun fichier spécifié")
            elif key == ord('h') or key == ord('H'):
                self.afficher_aide()
                print("\n Appuyez sur une touche dans la fenêtre pour continuer...")
                cv2.waitKey(0)
            
            rclpy.spin_once(self, timeout_sec=0.0)

        self.cap.release()
        cv2.destroyAllWindows()
        print("Fin")
    
    # ============================================================
    # COMMANDES FICHIER
    # ============================================================
    def executer_fichier_commandes(self, chemin_fichier):
        try:
            with open(chemin_fichier, 'r') as f:
                lignes = f.readlines()
            
            self.get_logger().info(f" Exécution du fichier : {chemin_fichier}")
            self.get_logger().info(f" {len(lignes)} lignes trouvées")
            
            self.wait_for_positions()
            time.sleep(0.5)
            
            for i, ligne in enumerate(lignes, 1):
                ligne = ligne.strip()
                if not ligne or ligne.startswith('#'):
                    continue
                
                self.get_logger().info(f"\n  Ligne {i}: {ligne}")
                self.executer_commande(ligne)
                time.sleep(0.2)
                rclpy.spin_once(self, timeout_sec=0.0)
            
            self.get_logger().info("\n Exécution du fichier terminée")
            
        except FileNotFoundError:
            self.get_logger().error(f" Fichier non trouvé : {chemin_fichier}")
        except Exception as e:
            self.get_logger().error(f" Erreur lors de l'exécution : {e}")
    
    def executer_commande(self, commande):
        parts = commande.split()
        if not parts:
            return
        
        cmd = parts[0].upper()
        args = parts[1:] if len(parts) > 1 else []
        
        try:
            if cmd == 'V':
                if len(args) >= 6:
                    vitesses = [float(x) for x in args[:6]]
                    self.get_logger().info(f"  ⚡ Vitesses : J0={vitesses[0]:.2f}, J1={vitesses[1]:.2f}, J2={vitesses[2]:.2f}, "
                                        f"J3={vitesses[3]:.2f}, J4={vitesses[4]:.2f}, J5={vitesses[5]:.2f}")
                    msg = Float64MultiArray()
                    msg.data = vitesses
                    self.arm_pub.publish(msg)
            
            elif cmd == 'H':
                self.get_logger().info("   Retour home...")
                if args and args[0] == '1':
                    self.go_home()
                else:
                    self.move_j0_and_wait(0.0)
                    self.move_j1_and_wait(0.0)
                    self.move_j2_and_wait(0.0)
                    self.move_j3_and_wait(0.0)
                    self.move_j4_and_wait(0.0)
                    self.move_j5_and_wait(0.0)
            
            elif cmd == 'I':
                self.get_logger().info("   Initialisation...")
                self.wait_for_positions()
                if self.current_arm_pos and self.current_wrist_pos:
                    self.get_logger().info(f"   Positions reçues :")
                    self.get_logger().info(f"     J0={self.current_arm_pos[0]:.1f}, J1={self.current_arm_pos[1]:.1f}, J2={self.current_arm_pos[2]:.1f}")
                    self.get_logger().info(f"     J3={self.current_wrist_pos[0]:.1f}, J4={self.current_wrist_pos[1]:.1f}, J5={self.current_wrist_pos[2]:.1f}")
            
            elif cmd == 'A':
                self.get_logger().info("   Alignement complet...")
                ret, frame = self.cap.read()
                if ret:
                    mask_cadre, _, _, _, _ = masque_cadre(frame)
                    pts_cadre, _ = plus_grand_quadrilatere(mask_cadre)
                    if pts_cadre is not None:
                        pts_cadre = self.aligner_complet(frame, pts_cadre, max_iterations=10)
                        if pts_cadre is not None:
                            _, pts_cadre = self.align_vertical_centre(frame, pts_cadre)
                            if self.current_arm_pos is not None:
                                self.j0_centre = self.current_arm_pos[0]
                                self.get_logger().info(f"   J0 centre enregistré : {self.j0_centre:.1f} mm")
                            
                            #  Mesure Z depuis le cadre
                            self.get_logger().info("   Mesure de Z depuis le cadre...")
                            z_mm = self.calculer_z_depuis_cadre(pts_cadre, frame.shape)
                            if z_mm is not None:
                                self.z_cam_actuel = z_mm
                                self.get_logger().info(f"   Z mesuré : {self.z_cam_actuel:.1f} mm")
                            else:
                                self.get_logger().warn("   Impossible de mesurer Z")
            
            elif cmd == 'D':
                if args:
                    try:
                        hauteur_cible = float(args[0])
                        self.get_logger().info(f"   Descente à {hauteur_cible:.1f} cm...")
                        self.descente_calculee()
                    except ValueError:
                        self.get_logger().error(f"   Hauteur invalide : {args[0]}")
            
            elif cmd == 'B':
                self.get_logger().info("   Navigation vers les barres...")
                #  VÉRIFICATION : Z doit être mesuré
                if self.z_cam_actuel is None:
                    self.get_logger().error("   Z non mesuré ! Impossible de naviguer vers les barres")
                    self.get_logger().error("   Faites d'abord A (alignement) ou Z (mesure)")
                    self.get_logger().error("   Commande B annulée")
                    return
                
                barres = []
                for arg in args:
                    if ',' in arg:
                        barres.extend([int(x.strip()) for x in arg.split(',') if x.strip().isdigit()])
                    else:
                        if arg.isdigit():
                            barres.append(int(arg))
                
                if barres:
                    self.liste_barres = barres
                    self.get_logger().info(f"   Barres sélectionnées : {self.liste_barres}")
                    self.navigation_vers_barres()
                else:
                    self.get_logger().error("  Aucun numéro de barre valide")
            
            elif cmd == 'W':
                if args:
                    try:
                        duree = float(args[0])
                        self.get_logger().info(f"    Attente de {duree:.1f} secondes...")
                        time.sleep(duree)
                    except ValueError:
                        self.get_logger().error(f"   Durée invalide : {args[0]}")
            
            elif cmd == 'P':
                if len(args) >= 6:
                    positions = [float(x) for x in args[:6]]
                    self.get_logger().info(f"   Position absolue : J0={positions[0]:.1f}, J1={positions[1]:.1f}, J2={positions[2]:.1f}, "
                                        f"J3={positions[3]:.1f}, J4={positions[4]:.1f}, J5={positions[5]:.1f}")
                    self.move_j0_and_wait(positions[0])
                    self.move_j1_and_wait(positions[1])
                    self.move_j2_and_wait(positions[2])
                    self.move_j3_and_wait(positions[3])
                    self.move_j4_and_wait(positions[4])
                    self.move_j5_and_wait(positions[5])
            
            elif cmd == 'M':
                if len(args) >= 6:
                    deltas = [float(x) for x in args[:6]]
                    self.get_logger().info(f"   Mouvement relatif : ΔJ0={deltas[0]:.1f}, ΔJ1={deltas[1]:.1f}, ΔJ2={deltas[2]:.1f}, "
                                        f"ΔJ3={deltas[3]:.1f}, ΔJ4={deltas[4]:.1f}, ΔJ5={deltas[5]:.1f}")
                    if self.current_arm_pos is not None:
                        self.move_j0(deltas[0])
                        self.move_j1(deltas[1])
                        self.move_j2(deltas[2])
                    if self.current_wrist_pos is not None:
                        self.move_j3(deltas[3])
                        self.move_j4(deltas[4])
                        self.move_j5(deltas[5])
            
            elif cmd == 'Z':
                self.get_logger().info("   Mesure de Z...")
                ret, frame = self.cap.read()
                if ret:
                    mask_cadre, _, _, _, _ = masque_cadre(frame)
                    pts_cadre, _ = plus_grand_quadrilatere(mask_cadre)
                    if pts_cadre is not None:
                        z_mm = self.calculer_z_depuis_cadre(pts_cadre, frame.shape)
                        if z_mm is not None:
                            self.z_cam_actuel = z_mm
                            self.get_logger().info(f"   Z = {self.z_cam_actuel:.1f} mm")
                        else:
                            self.get_logger().error("   Impossible de mesurer Z")
                    else:
                        self.get_logger().error("   Aucun cadre détecté")
            
            elif cmd == 'R':
                if args:
                    try:
                        angle = float(args[0])
                        self.get_logger().info(f"   Rotation J5 à {angle:.1f}°...")
                        self.move_j5_and_wait(angle)
                    except ValueError:
                        self.get_logger().error(f"   Angle invalide : {args[0]}")
            
            elif cmd == 'C':
                self.get_logger().info("   Positions actuelles :")
                if self.current_arm_pos is not None:
                    self.get_logger().info(f"     J0={self.current_arm_pos[0]:.1f} mm, J1={self.current_arm_pos[1]:.1f}°, J2={self.current_arm_pos[2]:.1f} mm")
                if self.current_wrist_pos is not None:
                    self.get_logger().info(f"     J3={self.current_wrist_pos[0]:.1f} mm, J4={self.current_wrist_pos[1]:.1f} mm, J5={self.current_wrist_pos[2]:.1f}°")
                if self.z_cam_actuel is not None:
                    self.get_logger().info(f"     Z={self.z_cam_actuel:.1f} mm")
            
            elif cmd == 'CHECK':
                self.afficher_diagnostic()
            
            elif cmd == 'S':
                if args:
                    fichier = args[0]
                    self.sauvegarder_configuration(fichier)
            
            elif cmd == 'L':
                if args:
                    fichier = args[0]
                    self.charger_configuration(fichier)
            
            elif cmd == 'HELP':
                self.afficher_aide()
            
            elif cmd == 'QUIT':
                self.get_logger().info("   Arrêt demandé")
                raise KeyboardInterrupt
            
            else:
                self.get_logger().warn(f"   Commande inconnue : {cmd}")
                
        except Exception as e:
            self.get_logger().error(f"   Erreur lors de l'exécution de '{commande}' : {e}")
    
    def sauvegarder_configuration(self, fichier):
        try:
            with open(fichier, 'w') as f:
                f.write("# Configuration du robot\n")
                f.write("# Format : COMMANDE arg1 arg2 ...\n\n")
                
                if self.current_arm_pos is not None and self.current_wrist_pos is not None:
                    f.write(f"P {self.current_arm_pos[0]:.1f} {self.current_arm_pos[1]:.1f} {self.current_arm_pos[2]:.1f} "
                        f"{self.current_wrist_pos[0]:.1f} {self.current_wrist_pos[1]:.1f} {self.current_wrist_pos[2]:.1f}\n")
                
                f.write(f"# J0 centre : {self.j0_centre:.1f}\n")
                f.write(f"# Barres à visiter : {self.liste_barres}\n")
                # B est commenté pour éviter les boucles
                
                if self.z_cam_actuel is not None:
                    f.write(f"# Z mesuré : {self.z_cam_actuel:.1f} mm\n")
                
                self.get_logger().info(f"   Configuration sauvegardée dans {fichier}")
        except Exception as e:
            self.get_logger().error(f"   Erreur lors de la sauvegarde : {e}")
    
    def charger_configuration(self, fichier):
        try:
            with open(fichier, 'r') as f:
                contenu = f.read()
            
            self.get_logger().info(f"   Configuration chargée depuis {fichier}")
            self.executer_fichier_commandes(fichier)
        except Exception as e:
            self.get_logger().error(f"   Erreur lors du chargement : {e}")
    
    def afficher_aide(self):
        aide = """
         COMMANDES DISPONIBLES :
        
        V 0.1 0.1 0.2 0.1 0.1 0.2    Vitesses (J0..J5)
        H 1                          Home 1 (tout à 0)
        I                            Initialisation (attente positions ROS)
        A                            Alignement complet + mesure Z
        D 2                          Descente à X cm
        B 1,2,3,4                    Navigation vers les barres
        W 5                          Attendre X secondes
        P 0 0 0 0 0 0                Position absolue (J0..J5)
        M 10 0 -5 0 0 0              Mouvement relatif (deltas)
        Z                            Mesurer la distance Z
        R 90                         Rotation J5 à X°
        C                            Afficher les positions actuelles
        CHECK                        Diagnostic complet
        S [fichier]                  Sauvegarder la configuration
        L [fichier]                  Charger une configuration
        HELP                         Afficher cette aide
        QUIT                         Quitter
        
         EXEMPLE DE FICHIER :
        
        # Initialisation
        I
        W 1
        
        # Alignement
        A
        W 2
        
        # Positions
        C
        P 100 10 20 30 40 0
        W 1
        C
        
        # Barres
        B 1,2,3
        W 2
        
        # Retour home
        H 1
        """
        self.get_logger().info(aide)


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = BarreDetectionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        print("Interrompu")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
