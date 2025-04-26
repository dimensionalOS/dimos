
This directory contains the pytest-based testing framework for the DIMOS project.


The test directory structure mirrors the main package structure:

```
tests/
├── agents/         # Tests for agent components
├── robot/          # Tests for robot components
├── skills/         # Tests for skill components
├── stream/         # Tests for stream components
├── web/            # Tests for web interface components
├── helpers/        # Standalone helper tests
├── deprecated/     # Old tests that are no longer maintained
├── conftest.py     # Pytest configuration and fixtures
└── test_header.py  # Import helper for proper path resolution
```


Tests can be run using the provided scripts:

```bash
./run_tests.sh

./run_tests.sh --module agents

./run_tests.sh --verbose

./run_tests.sh --coverage

./run_tests.sh --hardware
```

Alternatively, you can use the Python script:

```bash
python run_tests.py

python run_tests.py --module agents
```


When writing tests, follow these guidelines:

1. Place tests in the appropriate directory that mirrors the package structure
2. Import the test header at the beginning of each test file:
   ```python
   import tests.test_header
   ```
3. Use descriptive test names that clearly indicate what is being tested
4. Use pytest fixtures for common setup and teardown
5. Mock hardware dependencies for tests that don't require physical hardware
6. Use the `@pytest.mark.hardware` decorator for tests that require physical hardware
```
