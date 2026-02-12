#!/bin/bash
# AGV Web Control - Docker Startup Script

cd "$(dirname "$0")"

echo "=== AGV Web Control ==="
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    echo "Install Docker: https://docs.docker.com/engine/install/ubuntu/"
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: docker-compose is not installed"
    exit 1
fi

# Use docker compose (v2) or docker-compose (v1)
if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

case "$1" in
    build)
        echo "Building containers..."
        $COMPOSE_CMD build
        ;;
    up)
        echo "Starting containers..."
        $COMPOSE_CMD up -d
        echo ""
        echo "Services started!"
        echo "  - Backend:  http://localhost:8000"
        echo "  - Frontend: http://localhost:3000"
        echo "  - API Docs: http://localhost:8000/docs"
        ;;
    down)
        echo "Stopping containers..."
        $COMPOSE_CMD down
        ;;
    logs)
        $COMPOSE_CMD logs -f
        ;;
    restart)
        echo "Restarting containers..."
        $COMPOSE_CMD restart
        ;;
    *)
        echo "Usage: $0 {build|up|down|logs|restart}"
        echo ""
        echo "Commands:"
        echo "  build   - Build Docker images"
        echo "  up      - Start services"
        echo "  down    - Stop services"
        echo "  logs    - View logs"
        echo "  restart - Restart services"
        exit 1
        ;;
esac
