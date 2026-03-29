# AutoCol — Automatic Collision Box Generator for Blender

Génère des **collision boxes orientées (OBB)** qui épousent parfaitement
la forme d'un objet 3D — pas un simple AABB min/max.

---

## Description

| Mode | Résultat |
|---|---|
| **Single OBB** | Une seule box orientée sur les axes principaux d'inertie (PCA) |
| **Multi-OBB** | N boxes via K-means → couvre les objets complexes (L, T, croix…) |

La DLL C++ calcule la matrice de covariance pondérée par l'aire des triangles,
la diagonalise (algorithme de Jacobi), puis projette les vertices pour trouver
les extents exacts → **box la plus serrée possible**.

---

## Structure

```
autocol/
├── core/                     ← Code source C++
│   ├── CMakeLists.txt
│   ├── autocol_api.h         ← API C publique (extern "C")
│   ├── math3.h               ← Math 3D + Jacobi PCA (header-only, 0 deps)
│   ├── autocol_obb.cpp       ← Calcul OBB
│   ├── autocol_cluster.cpp   ← K-means multi-OBB
│   └── autocol_api.cpp       ← Exports DLL
├── addon/                    ← Add-on Blender Python
│   ├── __init__.py
│   ├── lib_loader.py         ← ctypes → DLL
│   ├── operators.py          ← Opérateurs Blender
│   ├── ui_panel.py           ← N-panel UI
│   └── autocol_core.dll      ← (généré par build.bat)
├── build.bat                 ← Script de compilation
└── README.md
```

---

## Prérequis

- **CMake** ≥ 3.15 → https://cmake.org/download/
- **Compilateur C++** (un des deux) :
  - **Visual Studio 2019 ou 2022** (Community, gratuit) avec "Desktop development with C++"
  - **MSYS2 + MinGW-w64** → https://www.msys2.org/
- **Blender** ≥ 3.6

---

## Installation

### Étape 1 — Compiler la DLL

```bat
cd c:\Users\Bonjour\Desktop\autocol
build.bat
```

Le script détecte automatiquement MSVC ou MinGW.
La DLL est copiée dans `addon\autocol_core.dll`.

### Étape 2 — Installer l'add-on dans Blender

1. Ouvrir Blender
2. `Edit > Preferences > Add-ons > Install…`
3. Naviguer jusqu'à `autocol\addon\` et sélectionner `__init__.py`
4. Activer **"AutoCol – Collision Box Generator"**

### Étape 3 — Utiliser

1. Sélectionner un mesh dans la vue 3D
2. Ouvrir le N-panel (`N`)
3. Aller dans l'onglet **AutoCol**
4. Choisir le mode (`Single OBB` ou `Multi-OBB`)
5. Cliquer **"Generate Collision Boxes"**
6. Les boxes apparaissent dans la collection `AutoCol_NomObjet`

---

## Résultat

- **Single OBB** : idéal pour les objets simples (capsule, cylindre, caisse)
- **Multi-OBB** : idéal pour les objets en L, T, ou avec des membres saillants
- Les boxes sont nommées `UCX_NomObjet_01`, `UCX_NomObjet_02`, etc.
  *(préfixe modifiable dans le panel — compatible Unreal Engine)*
- Affichage en **wireframe** par défaut pour ne pas masquer l'objet

---

## Algorithme (résumé)

```
1. Récupérer les triangles du mesh (avec modifiers)
2. Calculer la matrice de covariance C pondérée par l'aire (ΣwΔ·ΔᵀΔ)
3. Décomposer C par l'algorithme de Jacobi → 3 vecteurs propres = axes OBB
4. Projeter tous les vertices sur ces axes → min/max → extents + centre
5. (Multi-OBB) K-means sur les centres de faces → répéter 2-4 par cluster
```
