# SITL Utils - Drone Development Environment

Ambiente di sviluppo completo per droni autonomi con PX4, ROS2 Humble e architettura modulare per ricerca e sviluppo.

## 🎯 Architettura del Sistema

### Componenti Principali

```
sitl_utils/
├── 📦 ROS2 Packages (Submodules)
│   ├── traj_interp           - Interpolazione smooth traiettorie con controllo PX4
│   ├── path_planner          - Pianificazione percorsi 3D con OMPL
│   ├── drone_odometry2       - Fusione odometria e localizzazione
│   ├── teleop_node           - Controllo teleoperato
│   └── babyk_drone_manager   - Gestione stati e safety
├── 🎮 Simulazione
│   ├── models/               - Modelli Gazebo personalizzati
│   ├── worlds/               - Ambienti di simulazione
│   └── launch/               - File di avvio configurabili
├── 🐳 Docker Environment
│   ├── docker/               - Containerizzazione ROS2 + PX4
│   └── run_cnt.sh           - Script di avvio ambiente
└── ⚡ PX4 Integration
    ├── PX4-Autopilot/       - Firmware autopilota (escluso da git)
    └── bridge/              - Comunicazione ROS2 ↔ PX4
```

### 🚁 Trajectory Interpolator (traj_interp)

**Nodo principale per controllo autonomo dei droni PX4 con interpolazione smooth delle traiettorie.**

#### Caratteristiche Principali
- ✅ **Auto-Offboard**: Attivazione automatica modalità offboard all'avvio
- ✅ **Smart Arming**: Arm automatico solo al primo path o dopo atterraggio
- ✅ **Heading Auto**: Calcolo automatico yaw verso direzione di movimento
- ✅ **Smooth Interpolation**: Algoritmo ffilter con limitazioni jerk/accelerazione/velocità
- ✅ **Safety Features**: Disarm automatico all'atterraggio, gestione stati PX4

#### Topics Interface
```bash
# Input
/trajectory_path                 (nav_msgs/Path)         # Waypoints da seguire
/px4/odometry/out               (nav_msgs/Odometry)      # Feedback posizione

# Output  
/px4/trajectory_setpoint_enu    (MultiDOFJointTrajectory) # Setpoint interpolati
/fmu/in/offboard_control_mode   (OffboardControlMode)     # Controllo offboard
/fmu/in/vehicle_command         (VehicleCommand)          # Comandi arm/disarm
/trajectory_interpolator/status (std_msgs/String)        # Stato nodo
```

#### Quick Start
```bash
# 1. Avvia simulazione PX4
make px4_sitl gazebo_x500_depth

# 2. Avvia MicroDDS Agent
MicroXRCEAgent udp4 -p 8888

# 3. Lancia trajectory interpolator
ros2 launch traj_interp trajectory_interpolator.launch.py

# 4. Invia traiettoria di test
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 2.0, z: 5.0}, orientation: {w: 1.0}}}
  ]
}' --once
```

## 🚀 Setup Completo

### Prerequisites
- **Docker** con supporto GPU (per Gazebo)
- **Git** con configurazione SSH per submodules privati
- **16GB RAM** minimo raccomandato

### Installazione

1. **Clone Repository con Submodules**
   ```bash
   git clone --recursive https://github.com/Prisma-Drone-Team/sitl_utils.git
   cd sitl_utils
   
   # Se già clonato, sincronizza submodules
   git submodule update --init --recursive
   ```

2. **Setup PX4 Autopilot**
   ```bash
   # Clone PX4 firmware (release 1.14)
   git clone --single-branch -b release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive
   ```

3. **Build Docker Environment**
   ```bash
   cd docker
   docker build -t leo-img -f px4_humble_dockerfile.txt .
   ```

4. **Avvio Ambiente di Sviluppo**
   ```bash
   ./run_cnt.sh  # Avvia container con tutti i mount
   ```

### 🔧 Configurazione Workspace

All'interno del container:

```bash
# 1. Build ROS2 workspace
cd /root/ros2_ws
colcon build

# 2. Source environment
source install/setup.bash

# 3. Verifica installazione
ros2 pkg list | grep -E "(traj_interp|path_planner|teleop)"
```

## 🎮 Simulazione e Testing

### Avvio Simulazione Completa

```bash
# Terminal 1: PX4 SITL + Gazebo
cd /root/PX4-Autopilot
make px4_sitl gazebo_x500_depth

# Terminal 2: MicroDDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: ROS2 Nodes
ros2 launch traj_interp trajectory_interpolator.launch.py

# Terminal 4: Monitoring
ros2 topic echo /trajectory_interpolator/status
```

### Test Scenarios

#### 🛫 **Takeoff e Hover**
```bash
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [{header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}]
}' --once
```

#### 🔄 **Quadrato**
```bash
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 3.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 3.0, y: 3.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 3.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
  ]
}' --once
```

#### 🎯 **Landing**
```bash
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [{header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}]
}' --once
```

## 📊 Monitoring e Debug

### Status Monitoring
```bash
# Stati del trajectory interpolator
ros2 topic echo /trajectory_interpolator/status

# Setpoint inviati a PX4
ros2 topic echo /px4/trajectory_setpoint_enu

# Odometria drone
ros2 topic echo /px4/odometry/out

# Stato PX4
ros2 topic echo /fmu/out/vehicle_control_mode
```

### Visualizzazione
```bash
# RViz con configurazione pre-settata
ros2 run rviz2 rviz2 -d leo.rviz

# PlotJuggler per analisi real-time
ros2 run plotjuggler plotjuggler
```

## 🔧 Configurazione Parametri

### Trajectory Interpolator
File: `ros2_ws-src/pkg/traj_interp/config/trajectory_interpolator.yaml`

```yaml
# Performance Limits
ref_vel_max: 1.0          # Velocità massima [m/s]
ref_acc_max: 1.0          # Accelerazione massima [m/s²]  
ref_jerk_max: 2.0         # Jerk massimo [m/s³]

# Filter Tuning
ref_omega: 1.0            # Frequenza filtro [rad/s]
ref_zeta: 0.7             # Smorzamento

# Precision
waypoint_tolerance: 0.1   # Tolleranza raggiungimento waypoint [m]
control_frequency: 50.0   # Frequenza loop controllo [Hz]
```

## 🏗️ Sviluppo e Contribuzioni

### Struttura Submodules
Ogni package ROS2 è un submodule indipendente per permettere sviluppo modulare:

```bash
# Update specific submodule
git submodule update --remote ros2_ws-src/pkg/traj_interp

# Commit changes in submodule
cd ros2_ws-src/pkg/traj_interp
git add . && git commit -m "feat: new feature"
git push

# Update main repository
cd ../../..
git add ros2_ws-src/pkg/traj_interp
git commit -m "update: traj_interp submodule"
```

### Development Workflow
1. **Feature Branch**: Sviluppa in branch dedicati nei submodules
2. **Testing**: Testa in simulazione SITL
3. **Integration**: Aggiorna submodule nel main repository
4. **Documentation**: Aggiorna README con modifiche

## 🛡️ Safety Features

- ✅ **Auto-disarm**: Disarmo automatico all'atterraggio
- ✅ **Offboard Safety**: Controllo continuo stato offboard
- ✅ **Smooth Transitions**: Transizioni graduali tra modalità
- ✅ **Error Handling**: Gestione robusta errori comunicazione
- ✅ **Rate Limiting**: Limitazioni dinamiche sicure

## 📋 Troubleshooting

### Problemi Comuni

**🔴 Drone non si arma**
```bash
# Verifica stato land detector
ros2 topic echo /fmu/out/vehicle_land_detected

# Verifica offboard setpoints
ros2 topic hz /fmu/in/offboard_control_mode  # Deve essere ~50Hz
```

**🔴 Trajectory non seguita**
```bash
# Verifica ricezione path
ros2 topic echo /trajectory_path

# Verifica setpoint generati
ros2 topic echo /px4/trajectory_setpoint_enu
```

**🔴 Container non avvia**
```bash
# Verifica Docker e permissions
docker ps -a
sudo usermod -aG docker $USER  # Rilogin necessario
```

## 📚 References

- **PX4 Developer Guide**: https://docs.px4.io/main/en/development/
- **ROS2 Humble Docs**: https://docs.ros.org/en/humble/
- **Gazebo Classic**: http://gazebosim.org/tutorials
- **Docker Guide**: https://docs.docker.com/

## 📄 License

Questo progetto è rilasciato sotto licenza MIT - vedi [LICENSE.md](LICENSE.md) per dettagli.

---

**⚡ Sviluppato dal Team Prisma Drone per ricerca autonoma** 🚁
