#!/bin/bash

# Debug script to check paths in the container

echo "========================================="
echo "Debugging DimOS paths in container"
echo "========================================="
echo ""

echo "1. Checking DimOS directory:"
if [ -d "/workspace/dimos" ]; then
    echo "  ✓ /workspace/dimos exists"
    echo "  Contents:"
    ls -la /workspace/dimos/ | head -10
else
    echo "  ✗ /workspace/dimos NOT FOUND"
fi

echo ""
echo "2. Checking navigation directory:"
if [ -d "/workspace/dimos/dimos/navigation" ]; then
    echo "  ✓ /workspace/dimos/dimos/navigation exists"
    echo "  Contents:"
    ls -la /workspace/dimos/dimos/navigation/
else
    echo "  ✗ /workspace/dimos/dimos/navigation NOT FOUND"
fi

echo ""
echo "3. Checking rosnav directory:"
if [ -d "/workspace/dimos/dimos/navigation/rosnav" ]; then
    echo "  ✓ /workspace/dimos/dimos/navigation/rosnav exists"
    echo "  Contents:"
    ls -la /workspace/dimos/dimos/navigation/rosnav/ | head -10
else
    echo "  ✗ /workspace/dimos/dimos/navigation/rosnav NOT FOUND"
fi

echo ""
echo "4. Checking nav_bot.py:"
if [ -f "/workspace/dimos/dimos/navigation/rosnav/nav_bot.py" ]; then
    echo "  ✓ nav_bot.py exists"
    echo "  File info:"
    ls -la /workspace/dimos/dimos/navigation/rosnav/nav_bot.py
else
    echo "  ✗ nav_bot.py NOT FOUND"
fi

echo ""
echo "5. Checking Python virtual environment:"
if [ -d "/opt/dimos-venv" ]; then
    echo "  ✓ /opt/dimos-venv directory exists"
    if [ -f "/opt/dimos-venv/bin/python" ]; then
        echo "  ✓ venv Python exists"
        /opt/dimos-venv/bin/python --version
    else
        echo "  ✗ venv Python NOT FOUND"
    fi
else
    echo "  ✗ /opt/dimos-venv directory NOT FOUND"
fi

echo ""
echo "6. Environment variables:"
echo "  DIMOS_PATH=$DIMOS_PATH"
echo "  ROBOT_CONFIG_PATH=$ROBOT_CONFIG_PATH"
echo "  WORKSPACE=$WORKSPACE"
echo "  PWD=$PWD"

echo ""
echo "========================================="