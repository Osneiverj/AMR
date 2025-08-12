# Agents Overview

This file describes the “agents” of the AMR project, their responsibilities, technologies, and interfaces. It is kept in sync with the current Mission Orchestration plan in GEMINI.md.

| Agent               | Role / Responsibilities                                                                                                                                                                                                                                                | Technology / Package                                                                 | Interfaces                                                                                     |
|---------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| **tb3_sim**         | Robot Simulation Agent:<br>– Launches TurtleBot3 Gazebo world<br>– Provides simulated `/odom`, `/scan`, `/map` topics<br>– Hosts Nav2 and SLAM toolbox as needed<br>– Runs rosbridge websocket and the ROS Orchestrator package                                                                              | ROS 2 Humble, Gazebo 11, TurtleBot3 Gazebo packages                                                                       | ROS 2 DDS topics/services; WebSocket via rosbridge (`:9090`)                                     |
| **orchestrator**    | Mission Orchestration Agent:<br>– Provides `orchestrator/action/ExecuteMission` action server (`execute_mission`)<br>– Builds and runs Behavior Trees from mission JSON using `py-trees`<br>– Interacts with Nav2 (`NavigateToPose` action) to execute tasks<br>– Can manage SLAM/Nav2 lifecycle when required | rclpy, py-trees, py-trees-ros, nav2_msgs, lifecycle_msgs                                                                  | ROS 2 action server (`execute_mission`), ROS 2 nodes; launched within `tb3_sim` container                                         |
| **rosbridge_server**| Bridge Agent:<br>– Exposes ROS 2 graph over WebSocket for API/UI<br>– Enables clients to call actions/services and publish/subscribe                                                                                                                                  | `rosbridge_suite`                                                                                                         | WebSocket API (`ws://…:9090`)                                                                    |
| **rest_api**        | HTTP API Agent:<br>– CRUD for Maps, Points, Missions<br>– Validates missions with JSON Schema<br>– `/missions/{mid}/run` triggers mission execution by sending an action goal over rosbridge                                                                            | FastAPI, Beanie ODM, Motor (async Mongo), Pydantic, jsonschema, roslibpy, Uvicorn, python-multipart                        | HTTP REST (`http://…:5000`); connects to rosbridge (`AGV_ROSBRIDGE_URL`) to talk to orchestrator                                     |
| **mongodb**         | Database Agent:<br>– Persists maps metadata & filenames<br>– Stores points & missions documents                                                                                                                                                                       | MongoDB 7                                                                                                                 | Mongo wire protocol (`mongodb://…:27017`)                                                        |
| **ui_gateway**      | Web UI Agent:<br>– Dashboard + Mission Editor (React Flow)<br>– Create/Load/Save Missions; Run Mission button<br>– Displays live map/status via rosbridge                                                                                                             | React 18, Vite, React Flow, roslibjs, React-Leaflet, React-Select, Tailwind                                               | HTTP REST (API), WebSocket (rosbridge)                                                           |

## Runtime Topology

- `tb3_sim` container launches Gazebo world, Nav2, rosbridge websocket, and the `orchestrator` ROS 2 package via `system_launch.py`.
- `ui_gateway` serves the web app (Nginx) and talks to API (`VITE_API_URL`) and rosbridge (`VITE_ROSBRIDGE_URL`).
- `api` connects to MongoDB and to rosbridge (`AGV_ROSBRIDGE_URL`, defaults to `ws://tb3_sim:9090`).
- All services join the `ros_net` Docker network; published ports: `ui_gateway:8080`, `api:5000`, `tb3_sim:9090`.

## Communication Channels

- ROS 2 DDS: Simulation (`tb3_sim`) ↔ Orchestrator ↔ Nav2 / SLAM
- WebSocket (rosbridge): API/UI ↔ ROS graph (topics, services, actions)
- HTTP REST: UI ↔ API for maps, points, missions
- MongoDB: API ↔ MongoDB for persistence

## Mission Flow (Current)

1. User designs a mission in UI (React Flow) and saves it → API `/missions` (validated by `mission_schema.json`).
2. User clicks Run Mission → API `/missions/{mid}/run` uses `roslibpy` to send `ExecuteMission` goal over rosbridge.
3. Orchestrator builds a Behavior Tree from mission JSON and executes tasks (e.g., Nav2 `NavigateToPose`).
4. Feedback/result can be extended to stream back to UI (planned).

## Key Interfaces

- Orchestrator Action: `execute_mission` (type: `orchestrator/action/ExecuteMission`).
- Nav2 Action: `NavigateToPose` used by behaviors.
- Maps/Points/Missions REST: see `api` routers under `api/app/domain/...`.

## Status (from GEMINI.md)

- Completed:
  - API: Missions CRUD + `/missions/{mid}/run`; mission schema validation; seed script ready.
  - UI: Mission editor with React Flow; save/load/run; custom `NavigateToPose` node.
  - Orchestrator: `ExecuteMission` action server; dynamic BT building with `py-trees`; Nav2 integration.
  - Build: tb3_sim Dockerfile fixes; API and UI issues resolved.
- Next Steps:
  - End-to-end testing through simulation (UI → API → Orchestrator → Nav2).
  - Add advanced behaviors (timers, conditionals, composites).
  - Real-time feedback from BT execution to the UI.

## Extensibility

- New API module (e.g., Stations, Zones):
  - Create `api/app/domain/<module>/{model.py, service.py, router.py, schemas.py}`.
  - Add UI page at `ui_web/src/pages/<Module>.jsx` and client in `ui_web/src/services/...`.
- New mission behaviors: add BT behaviors in `turtlebot3-sim/orchestrator/orchestrator/behaviors.py` and map them from mission JSON.
- Multi-robot scaling: namespace topics/services and run multiple `tb3_sim` + orchestrator on distinct `ROS_DOMAIN_ID`s.

## Pointers (where things live)

- Orchestrator package: `turtlebot3-sim/orchestrator` (launches via `system_launch.py`).
- API missions: `api/app/domain/missions/*` (includes `mission_schema.json`).
- UI missions page: `ui_web/src/pages/Missions.jsx` and `components/NavigateToPoseNode.jsx`.
- Compose files: root `docker-compose.yml` and `turtlebot3-sim/docker-compose.yml`.
