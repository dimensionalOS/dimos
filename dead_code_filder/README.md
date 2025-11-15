# Dead Code Finder

A Python tool to analyze code dependencies and identify potentially unused code in the dimos project.

## Features

- Scans all Python files in the project (excluding specified directories like Detic)
- Extracts top-level function and class definitions
- Traces imports to their original source files (handles re-exports)
- Builds a comprehensive dependency graph
- Identifies potentially unused files and definitions

## Files

- `code_scanner.py` - Main scanner with enhanced import tracing
- `scanner.py` - Initial simpler version of the scanner
- `import_structure.json` - Generated dependency graph in JSON format
- `analysis.json` - Analysis results including dead code identification

## Output Format

The `import_structure.json` file contains entries for each Python file:

```json
{
  "path/to/file.py": {
    "defines": ["function1", "class1"],
    "uses": [
      ["source/file.py", "imported_function"],
      ["another/file.py", "ImportedClass"]
    ]
  }
}
```

- `defines`: List of top-level functions and classes defined in the file
- `uses`: List of [source_file, name] pairs showing what's imported from other files

## Import Tracing

The scanner properly traces imports to their original definition. For example:

```python
# file_a.py
def f(): ...

# file_b.py
from file_a import f

# file_c.py
from file_b import f  # Scanner knows this actually comes from file_a.py
```

## Running the Scanner

```bash
cd /home/p/pro/dimensional/dimos/dead_code_filder
python code_scanner.py
```

This will:
1. Scan all Python files in `/home/p/pro/dimensional/dimos`
2. Generate `import_structure.json` with the dependency graph
3. Generate `analysis.json` with dead code analysis
4. Print a summary of findings

## Analysis Results

The scanner identifies:
- Unused files (files with definitions that are never imported)
- Unused definitions (functions/classes that are never imported)
- Most used definitions
- Files with the most dependencies

Note: The scanner excludes test files, main scripts, and certain patterns from dead code detection as these are often entry points that won't be imported.

## D3 Visualization

The project includes an interactive D3.js visualization built in TypeScript that displays the dependency graph.

### Running the Visualization

1. First, ensure the Python scanner has been run to generate `import_structure.json`:
   ```bash
   python code_scanner.py
   # or
   npm run scan
   ```

2. Start the development server:
   ```bash
   npm run dev
   ```

3. Open your browser to http://localhost:3000

### Visualization Features

- **File Circles**: Each Python file is represented as a circle
  - Circle size varies based on the number of definitions
  - File name shown at the bottom
- **Definition Dots**: Each function/class is a gray dot inside the file circle
  - Shows truncated names (hover for full name)
  - Arranged in circular pattern within the file
- **Import Links**: Green lines show import relationships between files
- **Interactive Controls**:
  - Drag nodes to reposition them
  - Drag background to pan
  - Scroll to zoom in/out
  - Hover over dots to see full function/class names

### Color Scheme

- File circles: Dark gray (#404040)
- Definition dots: Gray (#808080)
- Import links: Green (#00ff00)
- Background: Light gray (#f5f5f5)

### Build for Production

To build the visualization for production:
```bash
npm run build
```

The built files will be in the `dist/` directory.