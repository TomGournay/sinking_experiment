# sinking_experiment

## Installation sur le robot (BlueOS / Raspberry Pi)

Python 3.9 ou plus récent. Exécuter depuis le dossier du projet sur le robot :

```bash
sudo apt-get update
sudo xargs -r -a requirements-system.txt apt-get install -y

git submodule update --init --recursive
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install -r requirements.txt

# Compiler INSLIB pour l'architecture du robot, même si un .so a été copié.
make -B -C INSLIB pylib

# Chart.js est nécessaire aux graphiques et son dossier est ignoré par Git.
mkdir -p frontend/libs
curl --fail --location https://cdn.jsdelivr.net/npm/chart.js@4.5.1/dist/chart.umd.js -o frontend/libs/chart.umd.js
```

`requirements.txt` contient les dépendances Python de l'application :
FastAPI, Uvicorn, NumPy, le pilote Navigator, SMBus2 et le pilote officiel
MS5837/Bar30. Pip installe aussi leurs dépendances transitives. Le pilote Bar30
provient du [dépôt Blue Robotics](https://github.com/bluerobotics/ms5837-python)
à une révision fixe ; le pilote Navigator est disponible sur
[PyPI](https://pypi.org/project/bluerobotics-navigator/).
Les outils INSLIB autonomes (interface Qt, GNSS, MAVLink, etc.) ne sont pas
utilisés par cette application et ne sont pas nécessaires ici.

`requirements-system.txt` contient les paquets système Debian/Raspberry Pi OS,
à installer avec APT, pas avec pip. `libopenblas0-pthread` fournit la bibliothèque
requise lorsque NumPy signale `libopenblas.so.0: cannot open shared object file`.
Une installation pip seule ne corrige pas cette bibliothèque manquante.

Vérifier les imports sans initialiser ni déplacer le robot :

```bash
PYTHONPATH="$PWD/INSLIB/python" python3 -c "import numpy, fastapi, uvicorn, smbus2, ms5837, bluerobotics_navigator, INSLIB; from sensors.calibration.accel_mag import AccelMagCalibration; print('Dépendances OK')"
python3 -m pip check
./launch.sh
```

À chaque nouvelle connexion au robot, activer le même environnement avant
de lancer l'application :

```bash
cd ~/sinking_experiment
. .venv/bin/activate
./launch.sh
```

Les fichiers de calibration et les enregistrements restent dans le projet.
Le pilote Navigator nécessite le robot et ses interfaces matérielles déjà
configurées ; installer les paquets ne configure pas les bus I2C/SPI.
Les paquets Python sont contraints par plages de versions, pas un verrouillage
complet de toutes les dépendances.

## Calibration

Le bouton existant calibre uniquement le biais du gyroscope (robot immobile).

Le bouton **Calibrer accéléromètre + magnétomètre** suit la procédure de
l'interface INSLIB (`tools/inslib_calib_gui.py`) :
1. Ne pas toucher le robot pendant les 20 premières secondes.
2. Soulever, tourner, poser et tenir environ 4 secondes dans chaque nouvelle
   orientation. Couvrir des orientations variées ; minimum solveur : 12 poses,
   20 ou plus conseillées.
3. Cliquer **Arrêter et calculer**, examiner les erreurs RMS avant/après et
   les avertissements INSLIB, puis **Enregistrer le résultat**.
   Annuler pendant la collecte ou abandonner le résultat conserve les valeurs actives.

Les pourcentages décrivent le repos initial puis l'objectif conseillé de
20 poses. Ils ne mesurent pas la qualité et ne bloquent pas le calcul.
Le compteur utilise directement `Recording.static_poses` ; le solveur
réévalue les intervalles avec son balayage de seuils. Les anciennes conditions
personnalisées (six secteurs, dispersion minimale, seuils RMS bloquants et
limite de 15 minutes) ont été supprimées. Les avertissements restent des
avertissements, comme dans INSLIB.

L'accéléromètre utilise `inslib_imu_tk.calibrate` avec `with_gyro=False`,
puis le résultat et les avertissements sont construits par INSLIB.
Comme dans son interface, l'estimation du désalignement est désactivée par
défaut (biais et échelles uniquement) et peut être activée dans les options.
Le magnétomètre utilise directement `inslib_imu_calib.solve_mag`, y compris
l'appariement temporel des poses et l'alignement magnétique sur l'accéléromètre.
La calibration gyro existante n'est ni recalculée ni modifiée.

Une erreur du solveur magnétique n'annule pas un résultat accéléromètre réussi.
La sauvegarde conserve alors l'ancienne correction magnétique et sa date,
ou n'applique aucune correction magnétique s'il n'en existait pas.
Les deux capteurs sont signalés séparément dans le statut.

Les références configurables sont celles d'INSLIB : gravité par défaut
9,80665 m/s² et champ magnétique 0 pour conserver l'échelle moyenne mesurée.
Une référence locale connue peut être saisie ; la position/WMM automatique
de l'outil INSLIB n'est pas connectée à cette application.
Le transport reste le pilote Navigator : chaque lecture IMU/magnétomètre
reçoit son horodatage hôte ; ce ne sont pas les horodatages matériels UBX
de l'outil INSLIB. La température du capteur n'est pas enregistrée ici.

Après validation par l'utilisateur, les corrections sont sauvegardées
atomiquement dans `accel_mag_calibration.json`, rechargées au démarrage et
appliquées comme `M @ (raw - bias)` avant la rotation vers le repère robot.
Les mesures brutes restent dans les journaux ; les champs `*_robot` sont
corrigés. Les métadonnées des enregistrements incluent la calibration active.

## Réutiliser après redémarrage

Les fichiers `gyro_calibration.json` et `accel_mag_calibration.json` sont
chargés automatiquement à chaque démarrage du backend. Il n'est pas nécessaire
de refaire les mouvements après un redémarrage. L'interface affiche pour
chaque capteur si une calibration est active, sa date et ses biais X/Y/Z ;
les matrices d'échelle et d'alignement sont aussi réappliquées.

Le bouton **Utiliser les dernières calibrations enregistrées** recharge les
dernières valeurs disponibles sur disque. Il fonctionne à l'arrêt, hors
calibration. Si un seul fichier existe, il est chargé ; les autres corrections
actives restent inchangées. Tous les fichiers disponibles sont validés avant
de remplacer les corrections actives. Aucun fichier de calibration n'est
modifié par ce bouton.

## Vérification

`python3 -m unittest discover -s tests -v`

Les tests utilisent des mesures synthétiques et des capteurs simulés pour
l'API ; NumPy et FastAPI sont nécessaires. La validation sur le robot reste
à effectuer.

## Tension et courant batterie (port POWER)

La tension (V) et le courant (A) s'affichent en haut à droite, y compris à l'arrêt et pendant les
calibrations. Le navigateur interroge `/api/battery` toutes les secondes
(après la fin de la requête précédente). Une erreur ou un délai de connexion
de 3 secondes remplace la valeur par « Indisponible ».

Le signal de tension du connecteur POWER (broche 4) est relié à ADC3 :
`navigator.read_adc(navigator.AdcChannel.Ch3)` renvoie la tension analogique
en volts. Avec le Power Sense Module Blue Robotics, la tension batterie est
cette valeur multipliée par **11,0**. Si le module est différent, régler
`BATTERY_VOLTAGE_MULTIPLIER` dans l'environnement du backend avant son lancement.
Pour ajuster la calibration : nouveau multiplicateur = multiplicateur actuel
× tension au multimètre / tension affichée.

Le courant est lu sur ADC2 (broche 3 du port POWER) et calculé suivant la
documentation du Power Sense Module : **I = (V_ADC2 − 0,330) × 37,8788 A**.
Les variables `BATTERY_CURRENT_MULTIPLIER` (A/V, défaut 37,8788) et
`BATTERY_CURRENT_OFFSET` (V, défaut 0,330) permettent d'adapter le module ou
sa calibration. Les valeurs signées sont conservées : un léger courant négatif
près de zéro peut refléter le bruit ou un décalage de calibration.
L'API renvoie `voltage` en volts et `current` en ampères.

Le port est analogique : il ne permet pas d'identifier automatiquement le module
ni de distinguer de façon fiable un câble débranché d'une tension nulle.
L'entrée de mesure accepte 0–3,3 V, pas la tension brute de la batterie.
La mesure est affichée en direct ; elle n'est pas ajoutée aux fichiers IMU/Bar30.

Sources Blue Robotics :
- [Branchement POWER](https://bluerobotics.com/learn/navigator-hardware-setup/)
- [Schéma Navigator, feuille 3](https://bluerobotics.com/wp-content/uploads/2022/06/NAVIGATOR-PCB-SCHEMATIC.pdf)
- [Power Sense Module, rapport 11 V/V](https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/)
- [Bibliothèque Navigator](https://github.com/bluerobotics/navigator-lib)
