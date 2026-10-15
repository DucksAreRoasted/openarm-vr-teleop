"""OpenArm desktop control UI (Qt)."""

__all__ = ["main"]


def main() -> None:
    from .main_window import run

    run()
