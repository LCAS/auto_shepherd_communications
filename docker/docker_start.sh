#!/bin/bash

# --- Configuration ---
COMPOSE_FILE="docker-compose.yml" # Your docker compose file name
SERVICE_NAME="auto_shepherd_communications_ros2_humble"         # The name of your service in docker-compose.yml

# Check if on Linux (most straightforward X11 forwarding)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Host OS: Linux. Setting up X11 permissions for Docker..."
    # Ensure DISPLAY is set (e.g., :0 or :1)
    if [ -z "$DISPLAY" ]; then
        echo "Error: DISPLAY environment variable is not set. Please ensure an X server is running and DISPLAY is configured."
        echo "Example: export DISPLAY=:0"
        exit 1
    fi
    # Allow local connections to the X server from Docker containers
    xhost +local:docker > /dev/null 2>&1 || {
        echo "Warning: xhost command failed. X11 forwarding might not work."
        echo "Ensure 'xauth' and 'x11-xserver-utils' are installed (e.g., 'sudo apt-get install xauth x11-xserver-utils')."
    }
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Host OS: macOS. X11 forwarding requires XQuartz."
    echo "Ensure XQuartz is installed and running."
    echo "You might need to enable 'Allow connections from network clients' in XQuartz preferences."
    echo "Consider passing '-e DISPLAY=host.docker.internal:0' in compose environment if issues persist."
elif [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "win32"* ]]; then
    echo "Host OS: Windows. X11 forwarding requires WSL2 and an X server like VcXsrv."
    echo "Ensure VcXsrv is running in 'Disable access control' mode."
    # For Windows WSL2, Docker Desktop automatically makes DISPLAY available via host.docker.internal
    # No direct 'xhost' needed on the Windows side.
    if [ -z "$DISPLAY" ]; then
        echo "Warning: DISPLAY environment variable is not set in WSL2. GUI apps may not work."
        echo "You might need to set 'export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf):0.0' in your WSL2 .bashrc."
    fi
else
    echo "Host OS: Unknown. X11 forwarding might not work as expected."
fi

# --- Build the Docker Compose services ---
echo "Building Docker Compose services..."
docker compose -f "${COMPOSE_FILE}" build "${SERVICE_NAME}"
if [ $? -ne 0 ]; then
    echo "Docker Compose build failed! Aborting."
    exit 1
fi
echo "Docker Compose build successful."

# --- Check and Stop/Remove existing container ---
echo "Running docker compose down to ensure a clean start..."
docker compose -f "${COMPOSE_FILE}" down --remove-orphans > /dev/null 2>&1 || true
echo "Docker Compose cleanup completed."

# --- Run the Docker Compose service interactively ---
echo "Starting Docker Compose service '${SERVICE_NAME}' in interactive mode..."

docker compose -f "${COMPOSE_FILE}" run --rm "${SERVICE_NAME}" bash

echo "Docker container session ended."
# If you want to run services in the background, you'd use 'docker compose up -d'
# and then 'docker compose exec SERVICE_NAME bash' to get a shell.
# But for a single interactive dev session, 'run --rm' is ideal.
