# AMR Development Workflow

Build the docker images once and then use compose overrides for live reload during development.

## Build base images

```bash
docker compose -f docker-compose.yml -f docker-compose.api.yml build
# build the ROS image
docker compose -f turtlebot3-sim/docker-compose.yml build
```

## Start development stack

```bash
# UI + API with live reload
docker compose up

# Simulation using the dev override to mount the orchestrator
docker compose -f turtlebot3-sim/docker-compose.yml -f turtlebot3-sim/docker-compose.dev.yml up
```

The overrides run the FastAPI server with `uvicorn --reload` and mount the
`orchestrator` package into the ROS workspace. The dev compose for the
simulation executes `colcon build` on start so any Python changes are
recompiled automatically.
