#!/bin/bash
# AGV Web Control - Frontend Startup Script

# Change to frontend directory
cd "$(dirname "$0")/frontend"

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install
fi

# Start development server
echo "Starting AGV Web Control frontend..."
npm run dev
