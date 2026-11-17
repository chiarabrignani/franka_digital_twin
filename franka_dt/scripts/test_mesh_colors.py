import xml.etree.ElementTree as ET
from pathlib import Path
import shutil


BASE = Path(__file__).resolve().parent.parent

TESTS = [
    (
        BASE / "meshes/pt/robots/fr3/visual/link1.dae",
        (0.1, 0.8, 0.2, 1.0),
        "PT verde"
    ),
    (
        BASE / "meshes/dt/robots/fr3/visual/link1.dae",
        (1.0, 0.45, 0.0, 0.45),
        "DT arancione"
    ),
]

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def modify_mesh(path, rgba, label):

    backup = path.with_suffix(".dae.bak")
    shutil.copy2(path, backup)

    tree = ET.parse(path)
    root = tree.getroot()

    diffuse_colors = root.findall(
        ".//c:profile_COMMON/c:technique/c:phong/c:diffuse/c:color",
        NS
    )

    if not diffuse_colors:
        diffuse_colors = root.findall(
            ".//c:profile_COMMON/c:technique/c:lambert/c:diffuse/c:color",
            NS
        )

    if not diffuse_colors:
        print(f"ATTENZIONE: nessun materiale trovato in {path}")
        shutil.copy2(backup, path)
        return

    for color in diffuse_colors:
        color.text = " ".join(map(str, rgba))

    tree.write(
        path,
        encoding="utf-8",
        xml_declaration=True
    )

    print(f"{label}: modificati {len(diffuse_colors)} materiali")


for path, rgba, label in TESTS:
    modify_mesh(path, rgba, label)
