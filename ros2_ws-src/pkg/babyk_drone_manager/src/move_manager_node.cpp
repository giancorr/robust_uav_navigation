#include "babyk_drone_manager/move_manager_node.h"
#include <iostream>
#include <thread>
#include <chrono>

MoveManagerNode::MoveManagerNode() 
    : Node("move_manager_node"),
      odometry_received_(false),
      running_(true),
      path_planner_status_("IDLE"),
      traj_interp_status_("IDLE"),
      overall_status_("IDLE"),
      teleop_active_(false),
      joy_available_(false),
      joy_ever_disconnected_(false),
      last_joy_time_(this->get_clock()->now()) {
    
    // Declare parameters
    declare_parameters();

    // Get parameters
    command_topic_ = this->get_parameter("command_topic").as_string();
    status_topic_ = this->get_parameter("status_topic").as_string();
    path_planner_goal_topic_ = this->get_parameter("path_planner_goal_topic").as_string();
    path_planner_status_topic_ = this->get_parameter("path_planner_status_topic").as_string();
    traj_interp_path_topic_ = this->get_parameter("traj_interp_path_topic").as_string();
    traj_interp_status_topic_ = this->get_parameter("traj_interp_status_topic").as_string();
    planned_path_topic_ = this->get_parameter("planned_path_topic").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    joy_topic_ = this->get_parameter("joy_topic").as_string();
    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    landing_altitude_ = this->get_parameter("landing_altitude").as_double();
    simulation_mode_ = this->get_parameter("simulation").as_bool();
    parent_frame_ = this->get_parameter("reference_frame").as_string();
    fixed_frame_ = this->get_parameter("fixed_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link").as_string();

    RCLCPP_INFO(get_logger(), "Move Manager Node initialized");
    RCLCPP_INFO(get_logger(), "  Command topic: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Takeoff altitude: %.2f m", takeoff_altitude_);
    RCLCPP_INFO(get_logger(), "  Landing altitude: %.2f m", landing_altitude_);
    RCLCPP_INFO(get_logger(), "  Simulation mode: %s", simulation_mode_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "  Reference frame: %s, Child frame: %s, Base link frame: %s", parent_frame_.c_str(), child_frame_.c_str(), base_link_frame_.c_str());

    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize TF2 for simulation if needed
    if (simulation_mode_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Timer for TF publishing at 100Hz
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&MoveManagerNode::timer_tf_callback, this));
        
        RCLCPP_INFO(get_logger(), "Simulation TF publishing enabled at 100Hz");
    }

    // Initialize publishers
    path_planner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        path_planner_goal_topic_, 10);
    
    traj_interp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        traj_interp_path_topic_, 10);
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);
    
    teleop_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/move_manager/teleop_active", 10);

    path_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/move_manager/path_mode", 10);

    seed_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/state", 10);

    cover_area_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/move_manager/cover_area", 10);
    
    cover_area_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/move_manager/cover_area_polygon", 10);

    // Initialize subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        command_topic_, 10, std::bind(&MoveManagerNode::command_callback, this, std::placeholders::_1));

    path_planner_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        path_planner_status_topic_, 10, std::bind(&MoveManagerNode::path_planner_status_callback, this, std::placeholders::_1));

    traj_interp_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        traj_interp_status_topic_, 10, std::bind(&MoveManagerNode::traj_interp_status_callback, this, std::placeholders::_1));

    interp_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/traj_interp/completed", 10, std::bind(&MoveManagerNode::interpolator_state_callback, this, std::placeholders::_1));

    planned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        planned_path_topic_, 10, std::bind(&MoveManagerNode::planned_path_callback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, qos, std::bind(&MoveManagerNode::odometry_callback, this, std::placeholders::_1));
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&MoveManagerNode::joy_callback, this, std::placeholders::_1));

    // Timer per controllare il timeout del joystick (controlla ogni 2 secondi)
    joy_timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2000), 
        std::bind(&MoveManagerNode::joy_timeout_callback, this));

    // Start command processor thread
    command_processor_thread_ = std::thread(&MoveManagerNode::command_processor, this);

    RCLCPP_INFO(get_logger(), "Move Manager Node ready to accept commands");
}

MoveManagerNode::~MoveManagerNode() {
    running_ = false;
    if (command_processor_thread_.joinable()) {
        command_processor_thread_.join();
    }
}

void MoveManagerNode::declare_parameters() {
    // Topic parameters
    this->declare_parameter("command_topic", "/seed_pdt_drone/command");
    this->declare_parameter("status_topic", "/move_manager/status");
    this->declare_parameter("path_planner_goal_topic", "/move_base_simple/goal");
    this->declare_parameter("path_planner_status_topic", "/path_planner/status");
    this->declare_parameter("traj_interp_path_topic", "/trajectory_path");
    this->declare_parameter("traj_interp_status_topic", "/trajectory_interpolator/status");
    this->declare_parameter("planned_path_topic", "/trajectory_path");
    this->declare_parameter("odometry_topic", "/px4/odometry/out");
    this->declare_parameter("joy_topic", "/joy");

    // Flight parameters
    this->declare_parameter("takeoff_altitude", 1.5);
    this->declare_parameter("landing_altitude", -0.5);
    
    // Simulation parameter
    this->declare_parameter("simulation", false);

    // Frame parameters
    this->declare_parameter("reference_frame", "drone/map");
    this->declare_parameter("child_frame", "odom");
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("fixed_frame", "map");
}

void MoveManagerNode::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_command_ = msg->data;
    RCLCPP_INFO(get_logger(), "Received command: %s", received_command_.c_str());
}

void MoveManagerNode::path_planner_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    path_planner_status_ = msg->data;
    update_overall_status();
}

void MoveManagerNode::traj_interp_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    traj_interp_status_ = msg->data;
    update_overall_status();
}

void MoveManagerNode::interpolator_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    interpolator_state_ = msg->data;
    if (interpolator_state_ == "armed"){
        std_msgs::msg::String seed_state_msg;
        seed_state_msg.data = "arm";
        seed_state_publisher_->publish(seed_state_msg);
    }
    else if (interpolator_state_ == "disarmed"){
        std_msgs::msg::String seed_state_msg;
        seed_state_msg.data = "-arm";
        seed_state_publisher_->publish(seed_state_msg);
    }
    // else if ((mode_msg_.data == "takeoff" || mode_msg_.data == "land") && interpolator_state_ == "traj_completed") {
    //     overall_status_ = (mode_msg_.data == "takeoff") ? "TAKEOFF_COMPLETED" : "LAND_COMPLETED";
    //     std_msgs::msg::String seed_state_msg;
    //     seed_state_msg.data = mode_msg_.data + ".done";
    //     seed_state_publisher_->publish(seed_state_msg);
    // }
}

void MoveManagerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr /*msg*/) {
    // Aggiorna il timestamp dell'ultimo messaggio joy ricevuto
    last_joy_time_ = this->get_clock()->now();
    
    // Detection del joystick - se arrivano messaggi, è disponibile
    if (!joy_available_) {
        joy_available_ = true;
        
        if (!joy_ever_disconnected_) {
            // Prima volta che rileva il joystick - tutto normale
            RCLCPP_INFO(get_logger(), "🎮 Joystick detected - teleop functionality enabled");
        } else {
            // Joystick ricollegato dopo disconnessione - NON riabilita teleop per sicurezza
            RCLCPP_WARN(get_logger(), "🎮 Joystick reconnected, but teleop remains DISABLED for safety");
            RCLCPP_WARN(get_logger(), "⚠️ Restart the move_manager node to re-enable teleop functionality");
        }
    }
}

void MoveManagerNode::joy_timeout_callback() {
    if (joy_available_) {
        auto now = this->get_clock()->now();
        auto time_since_last_joy = now - last_joy_time_;
        
        // Se non riceviamo messaggi joy per più di 3 secondi, considera il joy disconnesso
        if (time_since_last_joy.seconds() > 3.0) {
            joy_available_ = false;
            joy_ever_disconnected_ = true;  // Marca che il joy si è disconnesso almeno una volta
            RCLCPP_ERROR(get_logger(), "🚨 JOYSTICK DISCONNECTED - teleop functionality PERMANENTLY DISABLED");
            RCLCPP_ERROR(get_logger(), "⚠️ For safety, teleop will remain disabled even if joystick reconnects");
            RCLCPP_ERROR(get_logger(), "🔄 Restart move_manager node to re-enable teleop after reconnecting joystick");
            
            // Se eravamo in modalità teleop, usciamo automaticamente
            if (teleop_active_) {
                RCLCPP_ERROR(get_logger(), "🛑 EMERGENCY: Exiting teleop mode due to joystick disconnection");
                teleop_active_ = false;
                
                // Pubblica lo stato teleop_active = false
                auto teleop_msg = std_msgs::msg::Bool();
                teleop_msg.data = false;
                teleop_active_pub_->publish(teleop_msg);
                
                // Ferma il drone per sicurezza
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    overall_status_ = "EMERGENCY_STOPPED";
                }
                RCLCPP_ERROR(get_logger(), "🛑 Drone EMERGENCY STOPPED due to joystick disconnection");
            }
        }
    }
}

void MoveManagerNode::planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Solo inoltrare il percorso se stiamo aspettando un percorso pianificato
    // (evita loop infiniti di re-publishing)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (overall_status_ == "PLANNING_PATH") {
            RCLCPP_INFO(get_logger(), "Received planned path with %zu waypoints, forwarding to traj_interp", 
                        msg->poses.size());
            
            // Forward the planned path directly to traj_interp
            traj_interp_path_pub_->publish(*msg);
            
            // Aggiorna lo stato per indicare che il percorso è stato inoltrato
            overall_status_ = "PATH_FORWARDED";
        } else {
            RCLCPP_DEBUG(get_logger(), "Ignoring planned path (status: %s) - not expecting new path", 
                        overall_status_.c_str());
        }
    }
}

void MoveManagerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pose_ = msg->pose.pose;
    
    // Publish TF in simulation mode
    if (simulation_mode_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // Set the header
        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = child_frame_;
        transform_stamped.child_frame_id = base_link_frame_;

        // Set translation (position)
        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;

        transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    
    if (!odometry_received_) {
        odometry_received_ = true;
        RCLCPP_INFO(get_logger(), "First odometry received: [%.3f, %.3f, %.3f]", 
                    current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
    }
}

void MoveManagerNode::command_processor() {
    while (running_ && rclcpp::ok()) {
        std::string command_to_process;
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (current_command_ != received_command_ && !received_command_.empty()) {
                command_to_process = received_command_;
            }
        }
        
        if (!command_to_process.empty()) {
            std::vector<std::string> parts = parse_command(command_to_process);
            
            if (!parts.empty()) {
                // Stop any ongoing operation if new command received
                if (current_command_ != command_to_process) {
                    RCLCPP_INFO(get_logger(), "Processing new command: %s", command_to_process.c_str());
                    
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        current_command_ = command_to_process;
                    }
                    
                    process_command(parts);
                }
            }
        }
        
        // Publish status
        std_msgs::msg::String status_msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_msg.data = overall_status_;
        }
        status_pub_->publish(status_msg);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MoveManagerNode::process_command(const std::vector<std::string>& command_parts) {
    const std::string& command = command_parts[0];
    
    // Se è attivo il teleop e arriva un comando diverso da teleop, fermalo
    if (teleop_active_ && command != "teleop") {
        RCLCPP_INFO(get_logger(), "Stopping teleop to execute command: %s", command.c_str());
        teleop_active_ = false;
        
        // Disattiva la modalità teleop nel traj_interp
        std_msgs::msg::Bool teleop_flag;
        teleop_flag.data = false;
        teleop_active_pub_->publish(teleop_flag);
        RCLCPP_INFO(get_logger(), "Published teleop_active=false to restore normal trajectory control");
        
        // Usa la posizione corrente come ultima posizione teleop
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_teleop_pose_ = current_pose_;
            overall_status_ = "TELEOP_STOPPED";
        }
        
        // Piccola pausa per permettere al traj_interp di processare il cambio modalità
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::string path_mode = "flyto"; // default

    geometry_msgs::msg::PolygonStamped empty_poly;
    empty_poly.header.stamp = this->get_clock()->now();
    empty_poly.header.frame_id = fixed_frame_;
    // (non aggiungere punti)
    if (cover_area_polygon_pub_) {
        cover_area_polygon_pub_->publish(empty_poly);
    }
    
   if (command == "cover") {
        path_mode = "cover";
        mode_msg_.data = path_mode;
        path_mode_pub_->publish(mode_msg_);
        handle_cover_command(command_parts);
    } else if (command == "flyto") {
        if (command_parts.size() > 1) {
            if (command_parts[1].rfind("cover", 0) == 0) { // inizia con "cover"
                path_mode = "cover";
            } else if (command_parts[1].rfind("circle", 0) == 0) { // inizia con "circle"
                path_mode = "circle";
            }
            else if (command_parts[1].rfind("cover", 0) != 0 && command_parts[1].rfind("circle", 0) != 0) {
                path_mode = "flyto";
            }
        }
        mode_msg_.data = path_mode;
        path_mode_pub_->publish(mode_msg_);

        handle_flyto_command(command_parts);
    } else if (command == "go") {
        handle_go_command(command_parts);
    } else if (command == "takeoff") {
        handle_takeoff_command(command_parts);
        mode_msg_.data = "takeoff";
        path_mode_pub_->publish(mode_msg_);
    } else if (command == "land") {
        handle_land_command(command_parts);
        mode_msg_.data = "land";
        path_mode_pub_->publish(mode_msg_);
    } else if (command == "stop") {
        handle_stop_command();
    } else if (command == "teleop") {
        handle_teleop_command();
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown command: %s", command.c_str());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_UNKNOWN_COMMAND";
    }
}

void MoveManagerNode::handle_cover_command(const std::vector<std::string>& parts) {
    if (parts.size() < 2) {
        RCLCPP_ERROR(get_logger(), "cover command requires 4 corners: cover((x1,y1),(x2,y2),(x3,y3),(x4,y4))");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_COVER";
        return;
    }

    std::vector<std::pair<double, double>> corners = parse_corners_from_command(parts[1]);
    
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = this->now();
    polygon_msg.header.frame_id = fixed_frame_;

    for (const auto& [x, y] : corners) {
        geometry_msgs::msg::Point32 pt;
        pt.x = x;
        pt.y = y;
        pt.z = 1.5;
        polygon_msg.polygon.points.push_back(pt);
    }
    cover_area_polygon_pub_->publish(polygon_msg);

    if (corners.size() != 4) {
        RCLCPP_ERROR(get_logger(), "cover command requires exactly 4 corners");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_COVER";
        return;
    }

    // Trova il vertice con min x e min y (in basso a sinistra)
auto lower_left = std::min_element(
        corners.begin(), corners.end(),
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            return (a.first < b.first) || (a.first == b.first && a.second < b.second);
        }
    );
    size_t idx_ll = std::distance(corners.begin(), lower_left);

    // Trova i due vertici adiacenti
    size_t idx_next = (idx_ll + 1) % 4;
    size_t idx_prev = (idx_ll + 3) % 4;

    auto [x0, y0] = corners[idx_ll];
    auto [x1, y1] = corners[idx_next];
    auto [x3, y3] = corners[idx_prev];

    // Calcola i due lati adiacenti
    double dx1 = x1 - x0;
    double dy1 = y1 - y0;
    double dx2 = x3 - x0;
    double dy2 = y3 - y0;

    double len1 = hypot(dx1, dy1);
    double len2 = hypot(dx2, dy2);

    // Determina quale lato è più "orizzontale" (lungo x)
    double length, width;
    if (fabs(dx1) >= fabs(dx2)) {
        length = len1;
        width = len2;
    } else {
        length = len2;
        width = len1;
    }

    // Pubblica la zona di cover
    geometry_msgs::msg::Point cover_area;
    cover_area.x = length;
    cover_area.y = width;
    cover_area.z = 0.0;
    cover_area_pub_->publish(cover_area);

    // Crea la pose dello spigolo inferiore sinistro
    geometry_msgs::msg::PoseStamped spigolo_pose;
    spigolo_pose.header.stamp = this->get_clock()->now();
    spigolo_pose.header.frame_id = fixed_frame_; 
    spigolo_pose.pose.position.x = x0;
    spigolo_pose.pose.position.y = y0;
    spigolo_pose.pose.position.z = 1.5; // quota desiderata
    spigolo_pose.pose.orientation.w = 1.0; // nessuna rotazione

    // Trasforma la pose da "map" a "drone/map" se necessario
    try {
        geometry_msgs::msg::PoseStamped spigolo_pose_drone_map = tf_buffer_->transform(
            spigolo_pose, parent_frame_, tf2::durationFromSec(0.1));
        spigolo_pose = spigolo_pose_drone_map;
        spigolo_pose.header.frame_id = parent_frame_;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not transform spigolo pose from map to drone/map: %s", ex.what());
    }

    // Invia il goal al path planner
    path_planner_goal_pub_->publish(spigolo_pose);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "PLANNING_PATH";
    }

    RCLCPP_INFO(get_logger(), "Cover: starting frame=(%.2f,%.2f), length=%.2f, width=%.2f", x0, y0, length, width);
RCLCPP_INFO(get_logger(), "Published lower-left corner as start pose for path planner.");
}

std::vector<std::pair<double, double>> MoveManagerNode::parse_corners_from_command(const std::string& arg) {
    std::vector<std::pair<double, double>> corners;
    std::regex re(R"(\(([^,]+),([^)]+)\))");
    auto begin = std::sregex_iterator(arg.begin(), arg.end(), re);
    auto end = std::sregex_iterator();
    for (auto i = begin; i != end; ++i) {
        double x = std::stod((*i)[1]);
        double y = std::stod((*i)[2]);
        corners.emplace_back(x, y);
    }
    return corners;
}

void MoveManagerNode::handle_flyto_command(const std::vector<std::string>& parts) {
    if (parts.size() < 2) {
        RCLCPP_ERROR(get_logger(), "flyto command requires a frame name: flyto(frame_name)");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_FLYTO";
        return;
    }

    std::string frame = parts[1];
    frame_flyto_ = frame; // Store for later use

    // If cover(goal,X,Y) then extract goal, X, Y
    if (frame.rfind("cover", 0) == 0) {
        // frame is like "cover(goal,X,Y)"
        size_t open = frame.find('(');
        size_t close = frame.find(')', open);
        if (open != std::string::npos && close != std::string::npos && close > open) {
            std::string args = frame.substr(open + 1, close - open - 1);
            std::vector<std::string> tokens;
            size_t start = 0, end = 0;
            while ((end = args.find(',', start)) != std::string::npos) {
                tokens.push_back(args.substr(start, end - start));
                start = end + 1;
            }
            tokens.push_back(args.substr(start));
            // Rimuovi spazi
            for (auto& s : tokens) {
                s.erase(0, s.find_first_not_of(" \t"));
                s.erase(s.find_last_not_of(" \t") + 1);
            }
            if (tokens.size() >= 1) frame = tokens[0]; // goal
            if (tokens.size() >= 3) {
                try {
                    double X = std::stod(tokens[1]);
                    double Y = std::stod(tokens[2]);
                    geometry_msgs::msg::Point cover_area;
                    cover_area.x = X;
                    cover_area.y = Y;
                    cover_area.z = 0.0;
                    cover_area_pub_->publish(cover_area);

                    RCLCPP_INFO(get_logger(), "Parsed cover args: goal=%s, X=%.2f, Y=%.2f", frame.c_str(), X, Y);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Invalid X or Y in cover: %s", e.what());
                }
            }
        }
    } else {
        // Estrai il contenuto più interno tra parentesi, se presente (come già facevi)
        size_t last_open = frame.rfind('(');
        size_t first_close = frame.find(')', last_open);
        if (last_open != std::string::npos && first_close != std::string::npos && first_close > last_open) {
            frame = frame.substr(last_open + 1, first_close - last_open - 1);
        }
    }

    geometry_msgs::msg::Pose target_pose;
    if (lookup_transform(frame, target_pose)) {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->get_clock()->now();
        goal_msg.header.frame_id = parent_frame_;
        goal_msg.pose = target_pose;

        try {
            geometry_msgs::msg::PoseStamped goal_in_map = goal_msg;
            geometry_msgs::msg::PoseStamped goal_in_drone_map = tf_buffer_->transform(
                goal_in_map, parent_frame_, tf2::durationFromSec(0.1));
            goal_msg = goal_in_drone_map;
            goal_msg.header.frame_id = parent_frame_;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform goal from map to drone/map: %s", ex.what());
            // fallback: pubblica comunque in map
        }

        path_planner_goal_pub_->publish(goal_msg);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            overall_status_ = "PLANNING_PATH";
        }

        RCLCPP_INFO(get_logger(), "Sent flyto goal to path planner: [%.3f, %.3f, %.3f] (frame: %s)", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z, frame.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "Could not find transform for frame: %s", frame.c_str());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_FRAME_NOT_FOUND";
    }
}

void MoveManagerNode::handle_go_command(const std::vector<std::string>& parts) {
    if (parts.size() < 2) {
        RCLCPP_ERROR(get_logger(), "go command requires 3 coordinates: go(x,y,z)");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_GO";
        return;
    }

    // Split parts[1] by ','
    std::vector<std::string> coords;
    std::string s = parts[1];
    size_t start = 0, end = 0;
    while ((end = s.find(',', start)) != std::string::npos) {
        coords.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    coords.push_back(s.substr(start));

    if (coords.size() != 3) {
        RCLCPP_ERROR(get_logger(), "go command requires 3 coordinates: go(x,y,z)");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_GO";
        return;
    }

    try {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = std::stod(coords[0]);
        target_pose.position.y = std::stod(coords[1]);
        target_pose.position.z = std::stod(coords[2]);
        target_pose.orientation.w = 1.0; // Default orientation
        
        // Create direct path and send to traj_interp
        nav_msgs::msg::Path direct_path = create_direct_path(target_pose);
        traj_interp_path_pub_->publish(direct_path);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            overall_status_ = "EXECUTING_DIRECT_PATH";
        }
        
        RCLCPP_INFO(get_logger(), "Sent direct go command to traj_interp: [%.3f, %.3f, %.3f]", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid coordinates in go command: %s", e.what());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_COORDINATES";
    }
}

void MoveManagerNode::handle_takeoff_command(const std::vector<std::string>& /*parts*/) {
    if (!odometry_received_) {
        RCLCPP_ERROR(get_logger(), "Cannot takeoff: no odometry received");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_ODOMETRY";
        return;
    }

    geometry_msgs::msg::Pose takeoff_pose;
    bool used_tf = false;
    // Prova a usare la trasformazione in "map" tramite lookup_transform
    if (lookup_transform(base_link_frame_, takeoff_pose)) {
        RCLCPP_INFO(get_logger(), "Takeoff using TF transform (base_link->map): [%.3f, %.3f]", 
                    takeoff_pose.position.x, takeoff_pose.position.y);
        used_tf = true;
    } else {
        // Fallback: usa la logica attuale
        std::lock_guard<std::mutex> lock(state_mutex_);
        takeoff_pose = current_pose_;
        RCLCPP_WARN(get_logger(), "TF transform failed, takeoff from current odometry pose: [%.3f, %.3f]", 
                    takeoff_pose.position.x, takeoff_pose.position.y);
    }
    takeoff_pose.position.z = takeoff_altitude_;  // Only change altitude

    // Create SINGLE waypoint path for takeoff to avoid yaw calculation
    nav_msgs::msg::Path takeoff_path;
    takeoff_path.header.stamp = this->get_clock()->now();
    takeoff_path.header.frame_id = parent_frame_;
    
    // Add ONLY the target position (no start position to avoid horizontal movement calculation)
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header = takeoff_path.header;
    target_pose.pose = takeoff_pose;
    takeoff_path.poses.push_back(target_pose);
    
    traj_interp_path_pub_->publish(takeoff_path);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "TAKING_OFF";
    }

    RCLCPP_INFO(get_logger(), "Sent takeoff command to traj_interp: altitude %.3f (single waypoint)%s", 
                takeoff_altitude_, used_tf ? " [TF used]" : " [fallback]");
}

void MoveManagerNode::handle_land_command(const std::vector<std::string>& /*parts*/) {
    if (!odometry_received_) {
        RCLCPP_ERROR(get_logger(), "Cannot land: no odometry received");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_ODOMETRY";
        return;
    }
    
    geometry_msgs::msg::Pose land_pose;
    bool used_tf = false;
    // Prova a usare la trasformazione in "map" tramite lookup_transform
    if (lookup_transform(base_link_frame_, land_pose)) {
        RCLCPP_INFO(get_logger(), "Landing using TF transform (base_link->map): [%.3f, %.3f, %.3f]", 
                    land_pose.position.x, land_pose.position.y, land_pose.position.z);
        used_tf = true;
    } else {
        // Fallback: usa la logica attuale
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (last_teleop_pose_.position.x != 0.0 || last_teleop_pose_.position.y != 0.0 || last_teleop_pose_.position.z != 0.0) {
            land_pose = last_teleop_pose_;
            RCLCPP_INFO(get_logger(), "Landing from last teleop position: [%.3f, %.3f, %.3f]", 
                        land_pose.position.x, land_pose.position.y, land_pose.position.z);
        } else {
            land_pose = current_pose_;  // Keep current orientation
            RCLCPP_WARN(get_logger(), "TF transform failed, landing from current odometry pose: [%.3f, %.3f, %.3f]", 
                        land_pose.position.x, land_pose.position.y, land_pose.position.z);
        }
    }
    land_pose.position.z = landing_altitude_; // Use configurable landing altitude

    // Create SINGLE waypoint path for landing to avoid yaw calculation
    nav_msgs::msg::Path land_path;
    land_path.header.stamp = this->get_clock()->now();
    land_path.header.frame_id = parent_frame_;
    
    // Add ONLY the target position (no start position to avoid horizontal movement calculation)
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header = land_path.header;
    target_pose.pose = land_pose;
    land_path.poses.push_back(target_pose);
    
    traj_interp_path_pub_->publish(land_path);
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "LANDING";
    }
    
    RCLCPP_INFO(get_logger(), "Sent land command to traj_interp: altitude %.3f (single waypoint)%s", 
                landing_altitude_, used_tf ? " [TF used]" : " [fallback]");
}

void MoveManagerNode::handle_stop_command() {
    // Create empty path to stop trajectory execution
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = parent_frame_;
    
    traj_interp_path_pub_->publish(empty_path);
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "STOPPED";
    }
    
    RCLCPP_INFO(get_logger(), "Sent stop command");
}

std::vector<std::string> MoveManagerNode::parse_command(const std::string& command) {
    std::vector<std::string> result;

    auto parse_recursive = [&](const std::string& cmd, auto&& parse_recursive_ref) -> void {
        size_t paren_pos = cmd.find('(');
        if (paren_pos != std::string::npos) {
            // Nome comando
            std::string name = cmd.substr(0, paren_pos);
            result.push_back(name);

            // Trova parentesi chiusa corrispondente
            int depth = 0;
            size_t end_paren = std::string::npos;
            for (size_t i = paren_pos; i < cmd.size(); ++i) {
                if (cmd[i] == '(') depth++;
                else if (cmd[i] == ')') {
                    depth--;
                    if (depth == 0) {
                        end_paren = i;
                        break;
                    }
                }
            }
            if (end_paren != std::string::npos) {
                std::string args = cmd.substr(paren_pos + 1, end_paren - paren_pos - 1);

                // 🔹 Aggiungi l’argomento intero se è tipo "circle(goal)"
                result.push_back(args);

                // Se dentro ci sono altre parentesi, ricorsione
                parse_recursive_ref(args, parse_recursive_ref);
            }
        } else {
            // Nessuna parentesi → comando semplice o argomento base
            std::string trimmed = cmd;
            trimmed.erase(0, trimmed.find_first_not_of(" \t"));
            trimmed.erase(trimmed.find_last_not_of(" \t") + 1);
            if (!trimmed.empty()) {
                result.push_back(trimmed);
            }
        }
    };

    parse_recursive(command, parse_recursive);

    return result;
}

bool MoveManagerNode::lookup_transform(const std::string& frame_name, geometry_msgs::msg::Pose& pose) {
    try {
        geometry_msgs::msg::TransformStamped tf_stamped = 
            tf_buffer_->lookupTransform(parent_frame_, frame_name, tf2::TimePointZero);
        
        pose.position.x = tf_stamped.transform.translation.x;
        pose.position.y = tf_stamped.transform.translation.y;
        pose.position.z = tf_stamped.transform.translation.z;
        pose.orientation = tf_stamped.transform.rotation;
        
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Could not find transform from map to %s: %s", 
                    frame_name.c_str(), ex.what());
        return false;
    }
}

nav_msgs::msg::Path MoveManagerNode::create_direct_path(const geometry_msgs::msg::Pose& target_pose) {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = parent_frame_;
    
    // Add current position as start
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path.header;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        start_pose.pose = current_pose_;
    }
    path.poses.push_back(start_pose);
    
    // Add target position as end
    geometry_msgs::msg::PoseStamped end_pose;
    end_pose.header = path.header;
    end_pose.pose = target_pose;
    path.poses.push_back(end_pose);
    
    return path;
}

void MoveManagerNode::update_overall_status() {
    // Se il teleop è attivo, mantieni questo stato prioritario
    if (teleop_active_) {
        overall_status_ = "TELEOP_ACTIVE";
        return;
    }
    
    // Logic to combine statuses from path planner and traj interp
    if (path_planner_status_ == "PLANNING") {
        overall_status_ = "PLANNING_PATH";
    } else if (path_planner_status_.find("FAILED") != std::string::npos) {
        overall_status_ = "PLANNING_FAILED";
    } else if (traj_interp_status_ == "FOLLOWING_TRAJECTORY") {
        overall_status_ = "EXECUTING_TRAJECTORY";
        if (mode_msg_.data == "circle" && interpolator_state_ == "circle_run" ) {
            std_msgs::msg::String seed_state_msg;
            seed_state_msg.data = frame_flyto_ + ".running";
            seed_state_publisher_->publish(seed_state_msg);
        }
    } else if (traj_interp_status_ == "IDLE" && path_planner_status_ == "PATH_PUBLISHED" && overall_status_ != "PATH_FORWARDED") {
        overall_status_ = "TRAJECTORY_COMPLETED";  
        if (mode_msg_.data == "circle" || mode_msg_.data == "flyto") {
        std_msgs::msg::String seed_state_msg;
        seed_state_msg.data = frame_flyto_ + ".done";
        seed_state_publisher_->publish(seed_state_msg);
        }
        else if (mode_msg_.data == "cover"){
            if(interpolator_state_ == "cover_done"){
                std_msgs::msg::String seed_state_msg;
                seed_state_msg.data = current_command_ + ".done";
                seed_state_publisher_->publish(seed_state_msg);
            }
            else if ((interpolator_state_ == "cover_failed")){
                std_msgs::msg::String seed_state_msg;
                seed_state_msg.data = current_command_ + ".failed";
                seed_state_publisher_->publish(seed_state_msg);
            }
        }
    }
    // Manteniamo lo stato PATH_FORWARDED finché traj_interp non inizia a seguire la traiettoria
    // Keep other statuses as they are
}

// Timer callback for TF publishing in simulation mode
void MoveManagerNode::timer_tf_callback() {
    if (simulation_mode_) {
        static_tf_pub();
    }
}

// Function for static transforms (from original move_manager)
void MoveManagerNode::static_tf_pub() {
    geometry_msgs::msg::TransformStamped t;
    
    // Transform: base_link -> baby_k_0/OakD-Lite/base_link/IMX214
    // NOTE: For standard PX4 firmware, use x500_0/OakD-Lite/base_link/IMX214 instead of baby_k_0
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = base_link_frame_;
    t.child_frame_id = "baby_k_0/OakD-Lite/base_link/IMX214"; // For standard PX4: use "x500_0/OakD-Lite/base_link/IMX214"

    t.transform.translation.x = 0.2;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = -0.03;  // Camera at same height as base_link

    tf2::Quaternion q;
    q.setRPY(-1.5707, 0, -1.5707);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    static_tf_broadcaster_->sendTransform(t);

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = base_link_frame_;
    t.child_frame_id = "baby_k_0/OakD-Lite/base_link/StereoOV7251"; // For standard PX4: use "x500_0/OakD-Lite/base_link/IMX214"

    t.transform.translation.x = 0.2;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = -0.03;

    q.setRPY(-1.5707, 0, -1.5707);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    static_tf_broadcaster_->sendTransform(t);

    // Transform: base_link -> base_link_FRD  
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = base_link_frame_;
    t.child_frame_id = "base_link_FRD";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = -1.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;

    static_tf_broadcaster_->sendTransform(t);
}

void MoveManagerNode::handle_teleop_command() {
    // Verifica se il joystick è disponibile
    if (!joy_available_) {
        RCLCPP_ERROR(get_logger(), "❌ Cannot activate teleop mode - joystick not detected!");
        RCLCPP_ERROR(get_logger(), "Please ensure joystick is connected and joy_node is running");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_JOYSTICK";
        return;
    }
    
    // Verifica se il joystick si è mai disconnesso (safety lock)
    if (joy_ever_disconnected_) {
        RCLCPP_ERROR(get_logger(), "🚨 Cannot activate teleop mode - joystick was previously disconnected!");
        RCLCPP_ERROR(get_logger(), "⚠️ For safety, teleop is permanently disabled after any disconnection");
        RCLCPP_ERROR(get_logger(), "🔄 Please restart the move_manager node to re-enable teleop");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_TELEOP_SAFETY_LOCKED";
        return;
    }
    
    if (!odometry_received_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate teleop mode - no odometry received");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_ODOMETRY";
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Activating teleop mode - joystick control enabled");
    
    // PRIMA: Ferma la traiettoria attuale con path vuoto
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = parent_frame_;
    traj_interp_path_pub_->publish(empty_path);
    RCLCPP_INFO(get_logger(), "Sent empty path to stop current trajectory");
    
    // POI: Pubblica flag teleop_active per coordinare traj_interp
    std_msgs::msg::Bool teleop_flag;
    teleop_flag.data = true;
    teleop_active_pub_->publish(teleop_flag);
    RCLCPP_INFO(get_logger(), "Published teleop_active=true to coordinate trajectory_interpolator");
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        teleop_active_ = true;
        overall_status_ = "TELEOP_ACTIVE";
        
        // Stop any ongoing trajectory by setting status
        path_planner_status_ = "IDLE";
        traj_interp_status_ = "TELEOP_MODE";  // Nuovo stato
        
        // Reset and save current position as starting teleop position
        // Questo previene salti quando si rientra in teleop dopo disconnessione
        last_teleop_pose_ = current_pose_;
        RCLCPP_INFO(get_logger(), "🎯 Reset teleop reference to current position: [%.3f, %.3f, %.3f]", 
                    current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
    }
    
    RCLCPP_INFO(get_logger(), "Teleop mode activated - use joystick to control the drone");
    RCLCPP_INFO(get_logger(), "Send any other command (takeoff, land, go, etc.) to exit teleop mode");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
