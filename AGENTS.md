# Agents Overview

This file describes the “agents” that compose the AMR project, their responsibilities, technologies and interfaces.

| Agent               | Role / Responsibilities                                                                                                                             | Technology / Package                                  | Interfaces                                        |
|---------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|---------------------------------------------------|
| **tb3_sim**         | Robot Simulation Agent:<br>– Launches TurtleBot3 Gazebo world<br>– Provides simulated `/odom`, `/scan`, `/map` topics<br>– Hosts SLAM & Nav2 nodes   | ROS 2 (Humble), Gazebo, TurtleBot3 Gazebo packages    | ROS 2 DDS topics & services, exposed via rosbridge |
| **orchestrator**    | Lifecycle Orchestration Agent:<br>– Exposes `/ui/start_slam`, `/ui/stop_slam`, `/ui/load_map`, `/ui/start_nav2`, `/ui/stop_nav2` services<br>– Manages SLAM Toolbox & Nav2 lifecycle | ROS 2 Lifecycle, rclpy, lifecycle_msgs, nav2_msgs,<br> slam_toolbox_msgs | ROS 2 services                                   |
| **rosbridge_server**| Bridge Agent:<br>– Exposes ROS 2 over WebSocket (rosbridge)<br>– Enables web UI and external clients to subscribe/publish to ROS topics & call services | `rosbridge_suite`                                     | WebSocket API (`ws://…:9090`)                     |
| **rest_api**        | HTTP API Agent:<br>– CRUD for Maps, Points, Missions in MongoDB<br>– File‐upload endpoint for maps (`.pgm`/`.yaml`)<br>– Optional rosbridge proxy endpoints | FastAPI, Uvicorn, Beanie ODM, Motor (async Mongo), Pydantic, python-multipart | HTTP REST (`http://…:5000`)                      |
| **mongodb**         | Database Agent:<br>– Persists Maps metadata & filenames<br>– Stores Points & Missions documents<br>– Enables queries & updates from `rest_api`       | MongoDB 7                                              | Mongo wire protocol (`mongodb://…:27017`)         |
| **ui_gateway**      | Web UI Agent:<br>– React + Vite + Tailwind dashboard<br>– Displays live map, SLAM/Nav2 status, odometry, laser scan, footprint<br>– CRUD UI for data | React 18, Vite, roslibjs, React-Leaflet, React-Select  | HTTP REST, WebSocket (rosbridge)                  |

## Communication Channels

- **ROS 2 DDS**  
  Simulation (`tb3_sim`) ↔ Orchestrator ↔ SLAM Toolbox & Nav2  
- **WebSocket (rosbridge)**  
  Orchestrator & simulation expose ROS services/topics to the browser  
- **HTTP REST**  
  UI ↔ rest_api for persistent data (maps, points, missions)  
- **MongoDB**  
  rest_api ↔ mongodb for long‐term storage  

## Lifecycle

1. **Simulation** (`tb3_sim`) starts Gazebo, rosbridge and Orchestrator.  
2. **UI** connects to rosbridge (WebSocket) and to rest_api (HTTP).  
3. **User actions** in UI:  
   - **Create map** → calls `/ui/start_slam` via rosbridge or API proxy.  
   - **Save map** → calls SLAM service → uploads files to rest_api → MongoDB.  
   - **Activate map** → calls `/ui/load_map` & `/ui/start_nav2` → nav2 lifecycle.  
   - **CRUD points/missions** → HTTP calls to rest_api → MongoDB.  

## Extensibility

- **Adding a new module** (e.g. Stations, Zones) requires:  
  1. New domain folder in `api/app/domain/<module>` with `model.py`, `service.py`, `router.py`.  
  2. New page/component in `ui_web/src/pages/<Module>.jsx` + API wrapper in `services/api.js`.  
- **Custom ROS commands** can be exposed by adding services in the Orchestrator node.  
- **Scaling to multiple robots** by namespacing ROS topics/services and running multiple `tb3_sim` + orchestrator instances on different ROS_DOMAIN_IDs.

---
