# Implement pytest-based testing framework

This PR implements a pytest-based testing framework for the dimos project with the following features:

- Mirrored directory structure between source code and tests
- Test header import for proper path handling
- Unit tests for core components (Skills, Agents, video streaming)
- Integration test guidelines with mocking approach for hardware
- Standalone helper tests moved to a separate folder
- Executable run_tests.py script for convenient test execution

All tests can be run with `./run_tests.py` or directly with pytest.
