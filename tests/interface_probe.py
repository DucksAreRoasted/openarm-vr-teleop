"""Detect PR #4 pure-function declarations without compiling them."""

from __future__ import annotations

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]

MISSING_API = (
    "这个分支还没有 PR #4 的对应纯函数，所以先跳过。"
    "合并 https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/4 之后，"
    "同一条测试会自动编译并断言。本 PR 不必叠到 #4 上，也不必规定先合哪一个。"
)


def declares(relative: str, symbol: str, *, invocation: bool = True) -> bool:
    path = ROOT / relative
    if not path.is_file():
        return False
    text = path.read_text(encoding="utf-8")
    pattern = rf"\b{re.escape(symbol)}\s*\(" if invocation else rf"\b{re.escape(symbol)}\b"
    return re.search(pattern, text) is not None
