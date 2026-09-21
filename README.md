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

Le bouton **Calibrer accéléromètre + magnétomètre** lance une session commune
suivant le protocole actuel de `INSLIB/tools/inslib_imu_calib.py` :
1. Poser le robot et ne pas le toucher pendant 20 secondes.
2. Le tourner lentement autour des trois axes, le poser dans une nouvelle
   orientation et rester immobile environ 4 secondes. Répéter avec au moins
   20 poses variées : dessus, dessous, côtés et positions obliques.
   Les orientations exactes ne sont pas imposées.
3. Suivre le statut : nombre de poses détectées, secteurs couverts et
   couverture magnétique. Puis cliquer **Terminer et vérifier**.
   **Annuler la calibration** conserve les valeurs précédentes.

Le pourcentage global est une estimation des mouvements collectés : le minimum
du nombre de poses / 20, des secteurs couverts / 6, de la dispersion des
directions / 0,25 et de la dispersion magnétique / 0,25, plafonné à 95 %.
Le repos initial a son propre décompte. Les secteurs correspondent aux axes
signés de la Navigator ; ils servent à vérifier la couverture, sans imposer
un placement précis. Le compteur INSLIB en direct est indicatif : le solveur
réévalue les intervalles immobiles. 100 % signifie que les deux calibrations
ont été validées et enregistrées, pas simplement que du temps s'est écoulé.

La session utilise les solveurs INSLIB (accéléromètre : biais, échelle,
non-orthogonalité ; magnétomètre : fer dur, fer doux et alignement sur l'IMU).
Elle ne remplace pas la calibration gyro existante.
Les contrôles de qualité de l'application exigent au moins 20 poses, une
dispersion suffisante, un repos initial propre, une erreur accéléromètre
RMS <= 0,05 m/s², une erreur de norme magnétique <= 5 %, et un alignement
magnétique observable avec dispersion de l'inclinaison <= 3 degrés.
Une session est limitée à 15 minutes ; un échec demande de recommencer.

Les corrections sont enregistrées atomiquement dans
`accel_mag_calibration.json`, rechargées au démarrage et appliquées sous la
forme `M @ (raw - bias)` avant la rotation vers le repère robot.
Les mesures brutes restent dans les journaux ; les champs `*_robot` sont
corrigés. Les métadonnées de chaque enregistrement incluent la calibration.
L'enregistrement et les calibrations sont mutuellement exclusifs.

Aucune position géographique n'est configurée : la gravité de référence est
9,80665 m/s² et INSLIB conserve l'échelle magnétique moyenne mesurée
(`field_source: measured, no reference`), sans référence WMM locale.
Calibrer dans un champ stable, avec le robot assemblé et loin des objets
métalliques mobiles. Les pourcentages de couverture ne constituent pas une
mesure d'exactitude absolue.

Dépendance Python supplémentaire : NumPy (les solveurs viennent du sous-module
INSLIB présent dans ce dépôt). Lancer normalement avec `./launch.sh`.

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
