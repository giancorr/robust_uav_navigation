# vio_mapping

Implementazione ROS2/C++ della pipeline descritta nel documento di design:
mesh sparsa stile vio_mapping + poliedro di spazio libero + OVDE + DBOF,
alimentata da due istanze OpenVINS (front/rear), output come Octomap.

## Mappatura Step -> file

| Step | Cosa fa | File |
|---|---|---|
| 1) Receive landmarks | trasforma in world frame, fonde duplicati | `landmark_fusion.{hpp,cpp}` |
| 2) Build Sparse Visible Mesh | proiezione + Delaunay 2D + lifting 3D | `delaunay_mesh.{hpp,cpp}` |
| 3) Build Visible Free Space Polyhedron | coni camera->triangolo, intersezione raggi | `free_polyhedron.{hpp,cpp}` |
| 4) Open Area Virtual Depth (OVDE) | punti virtuali nei buchi di copertura | `ovde.{hpp,cpp}` |
| 5) Landmark Filtering (DBOF) | regole distanza/protezione | `dbof.{hpp,cpp}` |
| 6) Multi-Camera Fusion | implicito: front+rear fusi già in Step 1 e 3 | `landmark_fusion.cpp` (ingest per camera) + `free_polyhedron.cpp` (front_faces+rear_faces) |
| 7) Output (opzione B) | campionamento poliedro -> point cloud -> Octomap | `octomap_integration.{hpp,cpp}` |
| Nodo ROS2 | orchestrazione, subscriber/publisher | `vio_mapping_node.cpp` |

## Cosa devi adattare prima di compilare

1. **Tipo messaggio landmark reale.** Nel nodo ho lasciato un placeholder
   commentato (`onLandmarks`, `openvins_msgs::msg::Landmarks`): sostituiscilo
   con il tipo effettivo pubblicato dalla tua build di OpenVINS (verifica se
   pubblica covarianza per-landmark o solo posizione+id).

2. **Composizione TF imu->cam.** Il nodo assume che `/openvins_*/pose`
   pubblichi direttamente la posa che vuoi usare come camera pose. Se invece
   pubblica la posa IMU, devi comporre `T_world_cam = T_world_imu * T_imu_cam`
   usando la TF statica nota (`Timu_cam` dal tuo documento) prima di passare
   la posa a `DelaunayMesh::build` e a `fusion_.ingest(...)`.

3. **Intrinseci camera.** `intrinsics_front_`/`intrinsics_rear_` in
   `vio_mapping_node.cpp` sono placeholder: vanno sostituiti
   con la calibrazione reale del fisheye T265 (se lavori su immagine
   rettificata pinhole) o il proiettore va sostituito con un modello
   equidistant/Kannala-Brandt se proietti direttamente sul fisheye grezzo.

4. **Direction history per OVDE.** Ho lasciato `direction_history_front/rear`
   vuoti con TODO: vanno popolati a partire dalle `feature_tracks` di
   OpenVINS (che hai già nel topic `/openvins_*/features`), accumulando per
   ogni direzione la parallasse massima osservata nel tempo e se in quella
   direzione è mai stato rilevato un ostacolo. Questo storico è lo stato
   più "delicato" da implementare bene: ti consiglio una griglia di
   direzioni discretizzata in coordinate sferiche (az/el) ancorata al body
   frame, non al world frame, così ruota rigidamente col drone.

5. **Performance del merge in `LandmarkFusion::findMergeCandidate`.** È O(N)
   per ogni landmark ingerito: per molte centinaia di landmark per frame
   sostituiscilo con un kd-tree (es. nanoflann) o una hash grid spaziale.

6. **Subdiv2D in `DelaunayMesh::build`.** OpenCV `Subdiv2D` non ritorna
   direttamente gli indici dei punti inseriti, quindi nel codice si fa un
   match per coordinate (`find_id_by_point`, O(N) per triangolo). Va bene
   per decine/centinaia di landmark per camera; se superi qualche centinaio
   per frame, costruisci a monte una `std::unordered_map<pair<int,int>, id>`
   con coordinate quantizzate.

## Build

```bash
cd ~/ros2_ws/src
# copia/clona qui la cartella vio_mapping
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vio_mapping
source install/setup.bash
ros2 run vio_mapping vio_mapping_node
```

Dipendenze di sistema oltre a quelle ROS2 standard: `libopencv-dev`,
`libpcl-dev` (di solito già presenti se usi RTAB-Map), `liboctomap-dev`.

## Note sull'integrazione con il tuo planner basato su Octomap/RTAB-Map

- Il nodo pubblica sia `/octomap` (octomap_msgs/Octomap, compresso) sia
  `/mapping/debug_cloud` (PointCloud2 colorata free/occupied) per RViz.
- Dato che qui generiamo noi la point cloud "sintetica", **non serve**
  passare per il nodo `octomap_server` standard con raycasting da depth
  reale: l'inserimento avviene già con semantica corretta (free dai sample
  nel poliedro, occupied dai landmark filtrati DBOF). Se il tuo planner si
  aspetta comunque l'OcTree da `octomap_server`, puoi comunque alimentarlo
  con la point cloud di debug come se fosse una "depth cloud" sintetica,
  ma è ridondante: meglio consumare direttamente `/octomap` da qui.
- Per RTAB-Map: se lo usi solo per il grafo di pose/loop closure e non per
  l'occupancy, questo nodo può lavorare in parallelo prendendo le pose da
  OpenVINS (come ora) senza nessuna dipendenza da RTAB-Map. Se invece vuoi
  che RTAB-Map stesso consumi questa mappa, il punto di aggancio più
  semplice è pubblicare la point cloud di debug sul topic che RTAB-Map usa
  come "scan"/"cloud" di input per il suo occupancy grid 2D/3D.
