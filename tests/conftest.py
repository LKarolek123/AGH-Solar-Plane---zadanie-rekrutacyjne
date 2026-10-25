import sys
from pathlib import Path

# testy importują moduły z katalogu projektu
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
