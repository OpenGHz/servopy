"""Check documentation links, interface coverage and opt-in Python examples.

Run from any directory after installing .[docs]; --run also needs .[ruckig].
External URLs and unmarked code blocks are deliberately not executed.
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import fields
from html.parser import HTMLParser
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from urllib.parse import unquote, urlsplit

import markdown
from markdown.extensions.toc import slugify_unicode
import servo_py as sp


ROOT = Path(__file__).resolve().parents[1]
RUNNABLE = re.compile(
    r"^<!-- runnable: ([a-z0-9-]+) -->\s*\n```python\n(.*?)^```\s*$",
    re.MULTILINE | re.DOTALL,
)
REPO_PATH = re.compile(r"^/OpenGHz/servopy/(?:blob|tree)/main/(.*)$")


class Links(HTMLParser):
    def __init__(self, html: str):
        super().__init__()
        self.targets: list[str] = []
        self.anchors: set[str] = set()
        self.feed(html)

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        for name in ("href", "src"):
            if attributes.get(name):
                self.targets.append(attributes[name])
        if attributes.get("id"):
            self.anchors.add(attributes["id"])


def render_links(source: str) -> Links:
    return Links(markdown.markdown(
        source, extensions=["fenced_code", "tables", "toc"],
        extension_configs={"toc": {"slugify": slugify_unicode}},
    ))


def check_links(pages: dict[Path, str], errors: list[str]) -> int:
    parsed = {path: render_links(source) for path, source in pages.items()}
    count = 0
    for path, links in list(parsed.items()):
        for target in links.targets:
            url = urlsplit(target)
            repo_match = REPO_PATH.match(url.path) if url.netloc == "github.com" else None
            if repo_match:
                destination = ROOT / unquote(repo_match.group(1))
            elif url.scheme or url.netloc:
                continue
            else:
                destination = ((ROOT / unquote(url.path).lstrip("/")) if url.path.startswith("/")
                               else path.parent / unquote(url.path)) if url.path else path
            destination = destination.resolve()
            count += 1
            label = f"{path.relative_to(ROOT)}: {target}"
            if not destination.exists():
                errors.append(f"Missing link: {label}")
            elif destination.suffix == ".md" and url.fragment:
                if destination not in parsed:
                    parsed[destination] = render_links(destination.read_text(encoding="utf-8"))
                if unquote(url.fragment) not in parsed[destination].anchors:
                    errors.append(f"Missing anchor: {label}")
    return count


def check_interfaces(pages: dict[Path, str], errors: list[str]) -> tuple[int, int, int]:
    api = pages[ROOT / "docs/api.md"]
    status = pages[ROOT / "docs/status.md"]
    configuration = pages[ROOT / "docs/configuration.md"]
    # Read the source export list as well, so a stale installed package cannot
    # silently hide new public names from this documentation check.
    exports = next(ast.literal_eval(node.value) for node in ast.parse(
        (ROOT / "src/servo_py/__init__.py").read_text(encoding="utf-8")
    ).body if isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "__all__" for target in node.targets
    ))
    if set(exports) != set(sp.__all__):
        errors.append("Installed servo_py exports differ from source; reinstall the package")
    for name in exports:
        if not re.search(rf"\b{re.escape(name)}\b", api):
            errors.append(f"Undocumented public API: {name}")
    for enum in (sp.Action, sp.SafetyFlag):
        for name in enum.__members__:
            if f"`{name}`" not in status:
                errors.append(f"Undocumented {enum.__name__}: {name}")

    rows = dict(re.findall(r"^\| `([a-z_]+)` \| `([^`]+)` \|", configuration, re.MULTILINE))
    defaults = sp.ServoConfig()
    config_fields = fields(defaults)
    for field in config_fields:
        if field.name not in rows:
            errors.append(f"Undocumented ServoConfig default: {field.name}")
            continue
        try:
            actual = ast.literal_eval(rows[field.name])
        except (ValueError, SyntaxError):
            errors.append(f"Not a Python literal default: {field.name} = {rows[field.name]}")
            continue
        expected = getattr(defaults, field.name)
        if actual != expected:
            errors.append(f"Wrong default for {field.name}: documented {actual!r}, actual {expected!r}")
    for name in rows.keys() - {field.name for field in config_fields}:
        errors.append(f"Unknown ServoConfig parameter: {name}")
    return len(exports), len(config_fields), len(sp.SafetyFlag.__members__)


def check_cli(pages: dict[Path, str], errors: list[str]) -> int:
    source = pages[ROOT / "docs/cli.md"]
    sections = dict(re.findall(r"^## ([^\n]+)\n(.*?)(?=^## |\Z)", source, re.MULTILINE | re.DOTALL))
    scripts = {
        "mujoco_panda.py": "Panda 仿真",
        "track_pose.py": "位姿理想回放",
        "periodic_servo.py": "墙钟周期示例",
        "compare_references.py": "参考数值对照",
        "benchmark.py": "计算耗时基准",
    }
    count = 0
    for name, heading in scripts.items():
        tree = ast.parse((ROOT / "examples" / name).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "add_argument"):
                continue
            for arg in node.args:
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str) and arg.value.startswith("--"):
                    count += 1
                    if not re.search(rf"{re.escape(arg.value)}(?![\w-])", sections.get(heading, "")):
                        errors.append(f"Undocumented CLI argument: {name} {arg.value}")
    return count


def check_examples(pages: dict[Path, str], errors: list[str], run: bool) -> int:
    names: set[str] = set()
    for path, source in pages.items():
        examples = list(RUNNABLE.finditer(source))
        markers = re.findall(r"^<!-- runnable:.*?-->\s*$", source, re.MULTILINE)
        if len(markers) != len(examples):
            errors.append(f"Malformed runnable marker/block: {path.relative_to(ROOT)}")
        for match in examples:
            name, code = match.groups()
            if name in names:
                errors.append(f"Duplicate runnable name: {name}")
            names.add(name)
            label = f"{path.relative_to(ROOT)} ({name})"
            try:
                compile(code, str(path), "exec")
            except SyntaxError as exc:
                errors.append(f"Invalid Python in {label}: {exc}")
                continue
            if not run:
                continue
            # Match repo-relative model paths while keeping generated logs out
            # of the checkout. Each example gets its own process and directory.
            with tempfile.TemporaryDirectory(prefix="servo-docs-") as directory:
                workdir = Path(directory)
                (workdir / "examples").symlink_to(ROOT / "examples", target_is_directory=True)
                try:
                    result = subprocess.run([sys.executable, "-c", code], cwd=workdir,
                                            capture_output=True, text=True, timeout=30)
                except subprocess.TimeoutExpired:
                    errors.append(f"Example timed out after 30 s: {label}")
                    continue
            if result.returncode:
                errors.append(f"Example failed: {label}\n{result.stdout}{result.stderr}")
            else:
                print(f"PASS {name}: {result.stdout.strip()}", flush=True)
    return len(names)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Execute explicitly marked Python examples.")
    args = parser.parse_args()
    paths = sorted({*ROOT.glob("README*.md"), ROOT / "CONTRIBUTING.md", ROOT / "CHANGELOG.md",
                    *ROOT.glob("docs/**/*.md"), ROOT / "examples/assets/README.md"})
    pages = {path: path.read_text(encoding="utf-8") for path in paths}
    errors: list[str] = []
    links = check_links(pages, errors)
    exports, defaults, flags = check_interfaces(pages, errors)
    options = check_cli(pages, errors)
    examples = check_examples(pages, errors, args.run)
    if errors:
        for error in errors:
            print(f"ERROR {error}", file=sys.stderr)
        return 1
    print(f"Checked {len(pages)} pages, {links} local links, {exports} public names, "
          f"{defaults} defaults, {flags} flags, {options} CLI options and {examples} Python examples. "
          f"Examples {'executed' if args.run else 'syntax checked; use --run to execute'}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
