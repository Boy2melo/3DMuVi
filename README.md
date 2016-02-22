# 3DMuVi
*3D Reconstruction Framework from Multi-View Images*
<hr/>

**Betreuung:**

Bastian Erdnüß - <bastien.erdnuess@iosb.fraunhofer.de> <br>
Boitumelo Ruf - <boitumelo.ruf@iosb.fraunhofer.de> <br>
Fraunhofer Institut für Optronik, Systemtechnik und Bildauswertung (IOSB) - Abteilung Videoauswertesysteme (VID)
<hr/>

## \*\*\* NEWS \*\*\* ##

- **04.11.2015: Das erste Treffen findet am Donnerstag (05.11.2015) um 9:00 Uhr am Fraunhofer IOSB (Fraunhoferstr. 1) statt. BITTE bringen Sie ihre Ausweise mit und melden Sie sich bei Ankunft an der Pforte. Sie werden dann Besucherausweise erhalten.**<br/><br/>
- 03.11.2015: Zur Terminfindung für das erste Treffen bitte in Doodle eintragen: http://doodle.com/poll/xs6rtf234xue76g5 <br/>Termin wird dann hier bekannt gegeben.

<hr/>

## Kurzbeschriebung ##
In der bildbasierten 3D-Datenverarbeitung werden aus 2D-Bildmaterial (Foto/Video) 3D-Informationen gewonnen und verarbeitet. Hierzu soll eine intuitiv bedienbare GUI (C++/Qt5) mit Plugin System erstellt werden, die es erlaubt, die Zusammenarbeit verschiedener Algorithmen untereinander zu evaluieren.
<hr/>

## Hintergrund ##

Die 3D Rekonstruktion aus 2D Bildern ist eine inhärent schwierige Aufgabe zu der es (noch) keinen allumfassenden Lösungsalgorithmus gibt, der in allen Situationen optimale Ergebnise liefert.

State-of-the-Art ist, dass es eine Vielzahl von Algorithmen gibt, die gewisse Aspekte der 3D Rekonstruktion aufgreifen und lösen.  Daraus können Lösungsansätze zusammengestellt werden, die für die jeweilige Situation geeignete Ergebnisse liefern.

Dabei ist zum einen die Aufnahmekonfiguration zu beachten. So sind beispielsweise bei der Voraussicht aus einem fahrenden Auto  andere Algorithmen angebracht, als für Luftaufnahmen bei denen die Kamera rechtwinklig zur Flugrichtung zeigt. Wiederum sind andere Algorithmen notwendig, wenn man es mit unabhängigen Einzelbildern zu tun hat, die nicht aus einer Filmsequenz stammen.

Des Weiteren ist auch der Bildinhalt von Bedeutung.  Für Städterekonstruktion mit markanten Architekturen sind andere Algorithmen geeignet als beispielsweise für Naturlandschaftsrekonstruktion oder die Rekonstruktion von ausgedehnten Verkehrswegen.

#### Mögliche Anwendungsfälle
###### Workflow 1: Rekonstruktion aus unzusammenhängenden Einzelbilder (Building Rome in a Day)
*Gegeben:* Vielzahl an Einzelbilder einer Szene die nicht von der selben Kamera oder Position aufgenommen wurden.<br>
*Ziel:* Rekonstruktion der Szene oder eines einzelnen Objektes.

*Vorgehen:* <br>
1.	Feature Detektor: In den Bildern wird automatisch nach „markanten“ Punkten gesucht, die hohen Wiedererkennungswert haben.
2.	Feature Matching: Die gefundenen „markanten Punkte“ in zwei Bildern werden verglichen und einander zugeordnet.
3.	Posenschätzung: Aus den Punktzuordnungen von zwei oder mehr Bildern werden die Positionen der Kameras ermittelt, von denen aus die Bilder aufgenommen wurden.
4.	Tiefenschätzung: Mit Hilfe der Kamerapositionen werden die Tiefen in den Bildern trianguliert.  Dadurch werden die Entfernungen der Objekte auf den Bildern bestimmt.
5.	Modellerzeugung: Die Einzelansichten werden zu einem kompletten 3D Modell zusammengesetzt.

*Stellschrauben:* <br>
1.	Der Feature Detektor kann ausgewechselt werden (z. B. SIFT oder SURF)
2.	Das Feature Matching kann auf verschiedene Arten durchgeführt werden (z. B. RANSAC oder MLESAC, jeweils mit verschiedener Initialisierungen)
3.	usw.

###### Workflow 2: Structure-from-Motion (SfM)
*Gegeben:* Videosequenz einer Szene, mit eigenbewegung der Kamera.<br>
*Ziel:* Rekonstruktion der Szene oder eines einzelnen Objektes.

Structure-from-Motion (SfM) ist ein Verfahren zur 3D-Rekonstruktion mit nur einer Kamera. Dabei wird bei SfM das 3D-Modell  aus  einem  Video,  also  einer  Reihe  von  Einzelbildern  rekonstruiert.  Dabei  ist  es wichtig, dass das Video nicht von einer statischen Position aus aufgenommen wird, vielmehr sollte  die  Kamera  sich  dabei  mit  Blick  auf  die  zu  rekonstruierende  Szene  bewegen.  Bei  der Rekonstruktion mittels SfM werden zwei oder mehr Einzelbilder der Eingangssequenz herangezogen, für die angenommen wird, dass sie dieselbe Szene aus verschiedenen Blickrichtungen betrachten und die Szene zwischen den Einzelbildern statisch (unverändert) geblieben ist.

*Vorgehen:* <br>
1.	Feature Detection: Ermitteln leicht verfolgbarer markanter Bildpunkte
2.	Feature Tracking: Verfolgen der markanten Bildpunkte
3.	Posenschätzung: Bestimmen der Aufnahmepunkte, Verfolgen der Kamerafahrt
4.	Tiefenschätzung: Bestimmen der Tiefe mit lokalen Optimierungsverfahren
5.	Modellerzeugung: Die Einzelansichten werden zu einem kompletten 3D Modell zusammengesetzt

*Unterschiede zu Workflow 1:*<br>
-	Es werden andere Features benötigt.  Bei der Rekonstruktion aus unzusammenhängenden Einzelbilder müssen die Feature Punkte im anderen Bild anhand der Erscheinung wiedererkannt werden.  Bei SfM müssen sie nur lokal gut verfolgbar sein.
-	Tiefenschätzung: Bei SfM liegen aus der Viedosequenz immer Bilder vor, die aus beinahe derselben Perspektive aufgenommen wurden, mit einer optisch sehr ähnlichen Erscheinung.  Für die Tiefenschätzung können daher intensitätsbasierte Optimierungsverfahren verwendet werden.  Bei Rome in a Day hat man es in der Regel mit größeren Abständen zwischen den Aufnahmepositionen zu tun und unterschiedlichen Belichtungen (Tageszeiten, Jahreszeiten, etc.), so dass die Bilder sehr verschieden erscheinen.  Hier kann man nicht direkt auf den Intensitätswerten der Bilder arbeiten.

<hr/>

## Projektziel, Aufgabe und Anforderungen
In dem Projekt ***3DMuVi*** der *"Praxis der Softwareentwicklung"* soll ein Framework entwickelt werden mit dem das Zusammenspiel verschiedener Algorithmen evaluiert werden kann um eine geeignete Lösung für einen bestimmten Anwendungsfall zu finden.

Die Aufgabe umfasst Design, Implementierung und Dokumentation einer intuitiv bedienbaren Benutzeroberflaeche unter Beruecksichtigung verschiedener Workflows und Schnittstellen. Hierbei werden die Algortihmen über die entsprechenden Schnittstellen zur Verfügung gestellt und sollen flexibel mit einander verbunden werden können.

###### Anforderungen
- Das Framework soll Plugin-basiert aufgebaut werden, sodass verschiedene Algorithmen durch entsprechende Schnittstellen geladen werden können.
- Einzelnen Algorithmen sollen ausgewählt und zu einem Workflow verbunden werden.
- Laden von Eingangsdaten in Form von Einzelbilder oder Videos.
- Darstellung der Ergebnisse. Sowohl Entgültige, als auch Zwischenergebnisse (Feature Matches, Kamerapositionen / -bewegung, fertiges 3D-Modell).
- Interaktion mit den Ergebnissen. Beispielsweise das Drehen und Vergrößern des 3D-Modells, aber auch das korrigieren, auswählen und löschen von Kameras oder Features.

- (Speichern und laden gesamter Workflows)

<hr/>

## Software und Bibliotheken ##
 - C++11/14
 - [Qt5](http://http://www.qt.io/)
 - [OpenGL](https://www.opengl.org/)
 - [Point Cloud Library](http://pointclouds.org/)
 - [KITware MapTK](https://github.com/kitware/maptk)

## Schlagwörter für weitere Informationen ##
 - Structure-from-Motion
 - Building Rome in a Day
 - Monocular SLAM ([LSD-SLAM](http://vision.in.tum.de/research/vslam/lsdslam), [SVO](https://www.youtube.com/watch?v=2YnIMfw6bJY), [ORB-SLAM](http://webdiis.unizar.es/~raulmur/orbslam/), [PTAM](http://www.robots.ox.ac.uk/~gk/PTAM/), [DTAM](https://www.youtube.com/watch?v=Df9WhgibCQA))
 - Feature Deskriptoren (SIFT, SURF, ORB, BRIEF, FAST)
 - Epipolargeometrie
 - [Agisoft&trade;](http://www.agisoft.com)
 <hl/>

## Verwendete Bibliotheken ##

### VTK ###
[Homepage](http://www.vtk.org/)

#### License ####
VTK is an open-source toolkit licensed under the BSD license.

See LICENSE_VTK.txt for details.

### PCL ###
[Homepage](http://pointclouds.org/)

#### License ####
BSD License

See LICENSE_pcl.txt for details.

## Zusätzliche Bibliotheken kompilieren ##
Zum Kompilieren von 3D-MuVi werden VTK und pcl in ausreichend aktueller Version benötigt. Beide müssen mit Unterstützung für Qt5 kompiliert werden.

VTK kann unter diesem Link heruntergeladen werden:
[VTK-7.0.0] (http://www.vtk.org/files/release/7.0/VTK-7.0.0.tar.gz)

Von pcl wird die aktuelle Version aus dem git benötigt.
[pcl auf github] (https://github.com/PointCloudLibrary/pcl/)
