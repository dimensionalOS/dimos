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


@mock.patch('dimos.web.robot_web_interface.FastAPIServer')
def test_robot_web_interface_creation(mock_server_class):
    """Test that a RobotWebInterface can be created."""
    mock_server = mock.MagicMock()
    mock_server_class.return_value = mock_server
    
    mock_robot = mock.MagicMock()
    mock_agent = mock.MagicMock()
    
    web_interface = RobotWebInterface(
        robot=mock_robot,
        agent=mock_agent,
        port=5555
    )
    
    assert web_interface is not None
    assert web_interface.port == 5555


def test_fastapi_server_creation():
    """Test that a FastAPIServer can be created."""
    server = FastAPIServer()
    assert server is not None
    assert server.app is not None


def test_fastapi_server_routes():
    """Test the FastAPI server route setup."""
    server = FastAPIServer()
    
    assert server.app is not None
    assert hasattr(server, 'app')
    
    assert hasattr(server, 'query_subject')


@mock.patch('dimos.web.dimos_interface.api.server.rx')
def test_query_subject_creation(mock_rx):
    """Test that query subjects are created correctly."""
    mock_subject = mock.MagicMock()
    mock_rx.subject.Subject.return_value = mock_subject
    
    server = FastAPIServer()
    
    assert hasattr(server, 'query_subject')
    mock_rx.subject.Subject.assert_called()


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
        port=5555
    )
    
    mock_server.run = mock.MagicMock()
    
    web_interface.run()
    
    assert mock_server.run.called
