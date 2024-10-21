#!/bin/bash
IMAGE_NAME="epfl-lasa/ur_sim"
CONTAINER_NAME="${IMAGE_NAME//[\/.]/-}"
MODE=(interactive)
USE_NVIDIA_TOOLKIT=()
NO_GPU=false

# Help
HELP_MESSAGE="Usage: ./start_dockers.sh [interactive | server | connect] [-i, --image] [-u, --user] [--no-gpu]
Build the '${IMAGE_NAME}' image.
Options:
  interactive            Spin the image in the console
  server                 Spin the image as an ssh server
  connect                Connects to an active container
  -i, --image            The name of the image to use to start the container
  -u, --user             Specify the name of the login user. (optional)
  -h, --help             Show this help message and the one from aica-docker
  --no-gpu               Do not use the NVIDIA toolkit
  Additional arguments are passed to the aica-docker command.
  "

# Argument parsing
RUN_FLAGS=()
FWS_FLAGS=()
SHOW_HELP=false

while [ "$#" -gt 0 ]; do
    case "$1" in
    -i | --image)
        IMAGE_NAME=$2
        shift 2
        ;;
    -u | --user)
        USERNAME=$2
        shift 2
        ;;
    -m | --mode)
        MODE=$2
        shift 2
        ;;
    --no-gpu)
        NO_GPU=true
        shift 1
        ;;
    -h | --help)
        SHOW_HELP=true
        shift 1
        ;;
    *)
        if [ -z "${MODE}" ]; then
            MODE=$1
        else
            FWD_ARGS+=("$1")
        fi
        shift 1
        ;;
    esac
done



# Handle interactive/server specific arguments

# Check if a conitainer with this name is already running
if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} 2>/dev/null)" == "running" ]; then
echo "A container named ${CONTAINER_NAME} is already running. Stopping it."
docker stop ${CONTAINER_NAME}
fi

# Check if a NVIDIA GPU is available, if user want to use it and if the NVIDIA toolkit is installed
if [[ ($(sudo lshw -C display | grep vendor) =~ NVIDIA) && $NO_GPU == false ]]; then
USE_NVIDIA_TOOLKIT=true
echo "Detected NVIDIA graphic card, giving access to the container."
else
USE_NVIDIA_TOOLKIT=false
fi

# Handle GPU usage
[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

# Other
FWD_ARGS+=("--rm")
FWD_ARGS+=("-it")
FWD_ARGS+=("--net=host")
# FWD_ARGS+=("--ip=192.168.56.101")
# FWD_ARGS+=("-d universalrobots/ursim_cb3")
FWD_ARGS+=("-p")
FWD_ARGS+=("6080:6080")
FWD_ARGS+=("-e")
FWD_ARGS+=("ROBOT_MODEL=UR5")

#Add volume

docker volume rm programs
docker volume create --driver local \
--opt type="none" \
--opt device="${PWD}/programs" \
--opt o="bind" \
"programs"

FWD_ARGS+=(--volume="programs:/ursim/programs:rw")

# Setup git config
#FWD_ARGS+=(--volume="${HOME}/.gitconfig:/openpose/.gitconfig:ro")

# Start docker using aica
aica-docker \
    "${MODE}" \
    "${IMAGE_NAME}" \
    -n "${CONTAINER_NAME}" \
    ${GPU_FLAG} \
    "${FWD_ARGS[@]}" \
