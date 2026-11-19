#!/usr/bin/env python3
"""Locate a local MW2012 PC install without baking a home path into tools."""
import os
from pathlib import Path


GAME_NAME = "Need for Speed(TM) Most Wanted"


def _game_dirs():
    override = os.environ.get("NFS_MW_DIR")
    if override:
        path = Path(override).expanduser()
        if path.name.casefold() == "hawaii":
            return path.parent, path
        return path, path / "HAWAII"

    steam_roots = []
    if os.environ.get("STEAM_DIR"):
        steam_roots.append(Path(os.environ["STEAM_DIR"]).expanduser())
    home = Path.home()
    steam_roots.extend((
        home / ".local/share/Steam",
        home / ".steam/steam",
        home / ".steam/root",
        home / ".var/app/com.valvesoftware.Steam/.local/share/Steam",
    ))

    candidates = [root / "steamapps/common" / GAME_NAME for root in steam_roots]
    for game_root in candidates:
        hawaii = game_root / "HAWAII"
        if hawaii.is_dir():
            return game_root, hawaii

    # Keep a useful conventional default so the error can explain how to
    # configure a non-standard install when a pipeline command is run.
    game_root = candidates[0]
    return game_root, game_root / "HAWAII"


MW_ROOT, MW_DIR = _game_dirs()


def require_game_files():
    """Fail early with setup guidance if the retail HAWAII data is absent."""
    if not MW_DIR.is_dir():
        raise FileNotFoundError(
            "MW2012 game data not found. Set NFS_MW_DIR to the game's "
            "HAWAII folder (or its parent game folder)."
        )
