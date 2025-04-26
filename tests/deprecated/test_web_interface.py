#
#
#

"""Tests for the web interface components in the dimos package."""

import pytest
from unittest import mock
import json
from fastapi.testclient import TestClient

import tests.test_header

from dimos.web.robot_web_interface import RobotWebInterface
from dimos.web.dimos_interface.api.server import FastAPIServer


def test_robot_web_interface_creation():
    """Test that a RobotWebInterface can be created."""
    mock_robot = mock.MagicMock()
    mock_agent = mock.MagicMock()
    
    web_interface = RobotWebInterface(
        robot=mock_robot,
        agent=mock_agent,
        host="localhost",
        port=5555
    )
    
    assert web_interface is not None
    assert web_interface.host == "localhost"
    assert web_interface.port == 5555


def test_fastapi_server_creation():
    """Test that a FastAPIServer can be created."""
    server = FastAPIServer()
    assert server is not None
    assert server.app is not None


def test_fastapi_server_endpoints():
    """Test the FastAPI server endpoints."""
    server = FastAPIServer()
    client = TestClient(server.app)
    
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}
    
    response = client.get("/")
    assert response.status_code == 200


@mock.patch('dimos.web.dimos_interface.api.server.rx')
def test_submit_query(mock_rx):
    """Test submitting a query to the server."""
    mock_subject = mock.MagicMock()
    mock_rx.subject.Subject.return_value = mock_subject
    
    server = FastAPIServer()
    client = TestClient(server.app)
    
    response = client.post(
        "/api/submit_query",
        data={"query": "Test query"}
    )
    
    assert response.status_code == 200
    assert "success" in response.json()
    mock_subject.on_next.assert_called_once_with("Test query")


@mock.patch('dimos.web.robot_web_interface.FastAPIServer')
def test_robot_web_interface_run(mock_server_class):
    """Test running the web interface."""
    mock_server = mock.MagicMock()
    mock_server_class.return_value = mock_server
    
    mock_robot = mock.MagicMock()
    mock_agent = mock.MagicMock()
    
    web_interface = RobotWebInterface(
        robot=mock_robot,
        agent=mock_agent,
        host="localhost",
        port=5555
    )
    
    web_interface.run(blocking=False)
    
    mock_server_class.assert_called_once()
    mock_server.run.assert_called_once_with(blocking=False)
