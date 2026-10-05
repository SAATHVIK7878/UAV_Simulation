import sys
from pathlib import Path

# Make the package importable when running `pytest` from the project folder
# without installing it first.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
