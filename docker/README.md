# Docker

Due to the nature of the project, docker is used to add portability and compatibility on different platforms.

## Build the Docker containers

The docker containers are using docker compose tools with their profiles keys. Each parameter is define inside the docker-compose.yml file and can mount multiple container from the following profile list :

- **nvidia** - ros1 noetic with the tools to run the controller **with** nvidia support
- **intel** - ros1 noetic with the tools to run the controller **without** nvidia support
- **e-series** - run a polyscope window to interact with e-series UR robots [ur5e, ur10e, ...]
- **cb-series** - run a polyscope window to interact with cb-series UR robots [ur5, ur10, ...]

### Examples

Generic commands, autocompletion is supported for all of these commands :

```bash
# Building specific docker profile
docker compose --profile <profile_name> build

# Mounting specific docker profile in detached mode
docker compose --profile <profile_name> up -d

# Mounting specific docker profile in detached mode and build it if needed
docker compose --profile <profile_name> up -d --build

# Mounting multiple docker profile container by appending them, in detached mode with building option
docker compose --profile <profile_name> --profile <profile_name> up -d --build

# Accessing docker in interactive mode from a shell
docker exec -it <docker_container_name> bash
```

To run the nvidia support :

```bash
# Building ros1 noetic docker with nvidia support
docker compose --profile nvidia build

# Mounting ros1 noetic docker with nvidia support in detached mode
docker compose --profile nvidia up -d

# Mounting ros1 noetic docker with nvidia support in detached mode and build it if needed
docker compose --profile nvidia up -d --build

# Accessing docker in interactive mode from a shell
docker exec -it wp5-metal-additive-ros-1 bash
```
