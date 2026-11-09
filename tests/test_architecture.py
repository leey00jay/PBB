"""Enforce the dependency boundaries that let domain work merge independently."""

import ast
from importlib.util import resolve_name
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "pbb"
ALLOWED = {
    "perception": {"perception", "interfaces"},
    "planning": {"planning", "interfaces"},
    "control": {"control", "interfaces"},
    "interfaces": {"interfaces"},
    "adapters": {"adapters", "interfaces"},
}
AREAS = {*ALLOWED, "runtime"}


def boundary_errors(source, relative_path):
    """Check static Python imports, including relative and from-package imports."""
    path = Path(relative_path)
    area = path.parts[0] if len(path.parts) > 1 else "root"
    if area == "runtime":
        return []
    allowed = ALLOWED.get(area, set())
    package = ".".join(("pbb", *path.parent.parts))
    errors = []
    for node in ast.walk(ast.parse(source)):
        targets = []
        if isinstance(node, ast.Import):
            targets = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            target = node.module or ""
            if node.level:
                try:
                    target = resolve_name("." * node.level + target, package)
                except ImportError:
                    errors.append(f"line {node.lineno}: relative import escapes pbb")
                    continue
            targets = [target]
            targets.extend(f"{target}.{alias.name}" for alias in node.names)
        elif isinstance(node, ast.Call):
            name = node.func.id if isinstance(node.func, ast.Name) else ""
            if name in {"__import__", "eval", "exec"}:
                errors.append(f"line {node.lineno}: dynamic code is not allowed in {area}")
        for target in targets:
            parts = target.split(".")
            if parts[0] == "pbb" and len(parts) > 1 and parts[1] not in allowed:
                errors.append(f"line {node.lineno}: {area} must not import {target}")
            if parts[0] == "importlib":
                errors.append(f"line {node.lineno}: dynamic imports belong in runtime")
            if parts[0] == "carla" and area != "adapters":
                errors.append(f"line {node.lineno}: CARLA access belongs in adapters")
    return errors


class ArchitectureTests(unittest.TestCase):
    def test_package_dependencies(self):
        errors = []
        for path in sorted(PACKAGE.rglob("*.py")):
            relative = path.relative_to(PACKAGE)
            if len(relative.parts) > 1 and relative.parts[0] not in AREAS:
                errors.append(f"{relative}: define a dependency policy for this package")
            errors.extend(
                f"{relative}: {message}"
                for message in boundary_errors(path.read_text(encoding="utf-8"), relative)
            )
        self.assertEqual(errors, [], "\n".join(errors))

    def test_all_areas_are_discoverable_packages(self):
        for area in AREAS:
            with self.subTest(area=area):
                self.assertTrue((PACKAGE / area / "__init__.py").is_file())

    def test_rejects_cross_domain_imports(self):
        for source in (
            "import pbb.control.pid",
            "from pbb.control import pid",
            "from pbb import control",
            "from ..control import pid",
            "from .. import control",
        ):
            with self.subTest(source=source):
                self.assertTrue(boundary_errors(source, "planning/planner.py"))

    def test_allows_shared_and_same_domain_imports(self):
        source = "\n".join((
            "import math",
            "from pbb.interfaces import Observation",
            "from ..interfaces import Observation",
            "from .helpers import choose_path",
        ))
        self.assertEqual(boundary_errors(source, "planning/planner.py"), [])
        self.assertEqual(
            boundary_errors("from ...interfaces import Observation", "planning/route/solver.py"),
            [],
        )

    def test_shared_contracts_cannot_import_implementations(self):
        self.assertTrue(boundary_errors("from pbb.planning import Planner", "interfaces/data.py"))

    def test_adapters_cannot_invoke_algorithms(self):
        self.assertTrue(boundary_errors("from pbb.control import Controller", "adapters/carla.py"))
        self.assertEqual(boundary_errors("import carla", "adapters/simulator.py"), [])

    def test_runtime_can_compose_modules(self):
        source = "from pbb import perception, planning, control, adapters"
        self.assertEqual(boundary_errors(source, "runtime/pipeline.py"), [])

    def test_core_cannot_hide_dependencies_with_dynamic_imports(self):
        for source in ("import importlib", "from importlib import import_module", "__import__('pbb.control')"):
            with self.subTest(source=source):
                self.assertTrue(boundary_errors(source, "planning/planner.py"))

    def test_simulator_sdk_stays_outside_core(self):
        self.assertTrue(boundary_errors("import carla", "control/pid.py"))

    def test_root_package_cannot_reexport_implementations(self):
        self.assertTrue(boundary_errors("from .control import Controller", "__init__.py"))


if __name__ == "__main__":
    unittest.main()
