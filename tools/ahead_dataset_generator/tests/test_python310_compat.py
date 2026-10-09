import ast
from pathlib import Path


def test_python_sources_parse_as_python_310() -> None:
    root = Path(__file__).resolve().parents[1]
    sources = list((root / "src").rglob("*.py"))
    sources += list((root / "scripts").rglob("*.py"))
    sources += list((root / "tests").rglob("*.py"))
    for path in sources:
        source = path.read_text(encoding="utf-8")
        ast.parse(source, filename=str(path), feature_version=(3, 10))
