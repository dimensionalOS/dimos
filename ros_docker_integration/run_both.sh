#!/bin/bash
# Script to run both ROS route planner and DimOS together

echo "Starting ROS route planner and DimOS..."

# Variables for process IDs
ROS_PID=""
DIMOS_PID=""
SHUTDOWN_IN_PROGRESS=false

# Function to handle cleanup
cleanup() {
    if [ "$SHUTDOWN_IN_PROGRESS" = true ]; then
        return
    fi
    SHUTDOWN_IN_PROGRESS=true

    echo ""
    echo "Shutdown initiated. Stopping services..."

    # First, try to gracefully stop DimOS
    if [ -n "$DIMOS_PID" ] && kill -0 $DIMOS_PID 2>/dev/null; then
        echo "Stopping DimOS..."
        kill -TERM $DIMOS_PID 2>/dev/null || true

        # Wait up to 5 seconds for DimOS to stop
        for i in {1..10}; do
            if ! kill -0 $DIMOS_PID 2>/dev/null; then
                echo "DimOS stopped cleanly."
                break
            fi
            sleep 0.5
        done

        # Force kill if still running
        if kill -0 $DIMOS_PID 2>/dev/null; then
            echo "Force stopping DimOS..."
            kill -9 $DIMOS_PID 2>/dev/null || true
        fi
    fi

    # Then handle ROS - send SIGINT to the launch process group
    if [ -n "$ROS_PID" ] && kill -0 $ROS_PID 2>/dev/null; then
        echo "Stopping ROS nodes (this may take a moment)..."

        # Send SIGINT to the process group to properly trigger ROS shutdown
        kill -INT -$ROS_PID 2>/dev/null || kill -INT $ROS_PID 2>/dev/null || true

        # Wait up to 15 seconds for graceful shutdown
        for i in {1..30}; do
            if ! kill -0 $ROS_PID 2>/dev/null; then
                echo "ROS stopped cleanly."
                break
            fi
            sleep 0.5
        done

        # If still running, send SIGTERM
        if kill -0 $ROS_PID 2>/dev/null; then
            echo "Sending SIGTERM to ROS..."
            kill -TERM -$ROS_PID 2>/dev/null || kill -TERM $ROS_PID 2>/dev/null || true
            sleep 2
        fi

        # Final resort: SIGKILL
        if kill -0 $ROS_PID 2>/dev/null; then
            echo "Force stopping ROS..."
            kill -9 -$ROS_PID 2>/dev/null || kill -9 $ROS_PID 2>/dev/null || true
        fi
    fi

    # Clean up any remaining ROS2 processes
    echo "Cleaning up any remaining processes..."
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "localPlanner" 2>/dev/null || true
    pkill -f "pathFollower" 2>/dev/null || true
    pkill -f "terrainAnalysis" 2>/dev/null || true
    pkill -f "sensorScanGeneration" 2>/dev/null || true
    pkill -f "vehicleSimulator" 2>/dev/null || true
    pkill -f "visualizationTools" 2>/dev/null || true
    pkill -f "far_planner" 2>/dev/null || true
    pkill -f "graph_decoder" 2>/dev/null || true

    echo "All services stopped."
}

# Set up trap to call cleanup on exit
trap cleanup EXIT INT TERM

# Start ROS route planner in background (in new process group)
echo "Starting ROS route planner..."
cd /ros2_ws/src/autonomy_stack_mecanum_wheel_platform
setsid bash -c './system_simulation_with_route_planner.sh' &
ROS_PID=$!

# Wait a bit for ROS to initialize
echo "Waiting for ROS to initialize..."
sleep 5

# Start DimOS
echo "Starting DimOS Unitree G1 controller..."
python /home/p/pro/dimensional/dimos/dimos/navigation/rosnav/nav_bot.py &
DIMOS_PID=$!

echo "Both systems are running. Press Ctrl+C to stop."
echo ""

# Wait for both processes
wait $ROS_PID $DIMOS_PID 2>/dev/null || true