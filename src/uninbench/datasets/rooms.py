"""IF3: indoor rooms (Bormann et al., 2016/2018) with and without furniture.

Input maps are occupancy images (``<name>.png``, free = white), their
furnished variants (``<name>_furnitures.png``) and optionally a ground-truth
room segmentation (``<name>_gt_segmentation.png``) in which rooms are white
regions separated by black lines.  Every room yields a *paired* pair of
instances -- the empty room and the same room with furniture -- so that the
effect of clutter can be measured per room.  Pixel regions are vectorised
exactly (union of pixel squares, optionally simplified by half a pixel) and the
source resolution is stored with the environment, since it bounds the fidelity
any adapter can achieve.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import shapely
from PIL import Image
from scipy import ndimage
from shapely.geometry import Point
from shapely.ops import nearest_points

from ..geometry.decomposition import polygons_of
from ..geometry.footprint import Footprint
from ..geometry.raster import RasterGrid, mask_to_geometry
from ..instance.model import Environment, Instance, Robot


def load_free(path, threshold: int = 250) -> np.ndarray:
    a = np.asarray(Image.open(path).convert("L"))
    return a >= threshold


def _to_geometry(mask: np.ndarray, res: float, simplify: float):
    grid = RasterGrid(0.0, 0.0, res, mask.shape[1], mask.shape[0])
    g = mask_to_geometry(grid, mask[::-1])  # image row 0 is the top
    if simplify > 0:
        g = g.simplify(simplify, preserve_topology=True)
    return shapely.make_valid(g)


def _instance(iid, geom, res, width, variant, room, meta):
    polys = [p for p in polygons_of(geom)]
    boundary = shapely.unary_union([shapely.Polygon(p.exterior) for p in polys])
    holes = [shapely.Polygon(h) for p in polys for h in p.interiors]
    env = Environment(boundary=boundary, holes=holes, source_resolution=res)
    body = Footprint("disc", width)
    cs = body.erode(env.free)
    comps = polygons_of(cs)
    if not comps:
        return None
    main = max(comps, key=lambda c: c.area)
    c = main.representative_point()
    q = nearest_points(main.buffer(-1e-6 * width), Point(c.x, c.y))[0]
    robot = Robot(id="r0", start=(q.x, q.y, 0.0), tool=body)
    m = {"source": "rooms", "room": room, "variant": variant, "factors": {"furniture": variant == "furnished"},
         "pair": room, **meta}
    return Instance(id=iid, environment=env, team=[robot], family="IF3", metadata=m)


def import_room_map(map_png, furnished_png=None, segmentation_png=None, resolution: float = 0.05,
                    width: float = 0.5, min_room_area: float = 4.0, simplify: float | None = None, out_dir=None):
    map_png = Path(map_png)
    name = map_png.stem
    free = load_free(map_png)
    furn = load_free(furnished_png) & free if furnished_png else None
    simplify = 0.5 * resolution if simplify is None else simplify
    if segmentation_png:
        seg_free = load_free(segmentation_png) & free
        labels, n = ndimage.label(seg_free, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    else:
        labels, n = ndimage.label(free, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    out = []
    px_area = resolution * resolution
    sizes = ndimage.sum(np.ones_like(labels), labels, index=np.arange(1, n + 1))
    k = 0
    for lab in range(1, n + 1):
        if sizes[lab - 1] * px_area < min_room_area:
            continue
        room = labels == lab
        rid = f"{name}_room{k:02d}"
        k += 1
        variants = [("empty", room)]
        if furn is not None:
            variants.append(("furnished", room & furn))
        for variant, mask in variants:
            geom = _to_geometry(mask, resolution, simplify)
            if geom.is_empty:
                continue
            inst = _instance(f"{rid}_{variant}", geom, resolution, width, variant, rid,
                             {"map": map_png.name, "segmentation": bool(segmentation_png)})
            if inst is None:
                continue
            out.append(inst)
            if out_dir is not None:
                inst.save(Path(out_dir))
    return out


def import_rooms(src, out_dir=None, resolution: float = 0.05, width: float = 0.5, min_room_area: float = 4.0,
                 limit: int | None = None):
    """Import every ``<name>.png`` with a ``<name>_furnitures.png`` variant in a directory."""
    src = Path(src)
    out = []
    maps = sorted(p for p in src.glob("*.png") if not any(s in p.stem for s in ("_furnitures", "_gt_segmentation",
                                                                                 "_trashbins")))
    for m in maps[:limit] if limit else maps:
        furn = src / f"{m.stem}_furnitures.png"
        seg = src / f"{m.stem}_gt_segmentation.png"
        if not furn.exists():
            continue
        out += import_room_map(m, furn, seg if seg.exists() else None, resolution, width, min_room_area,
                               out_dir=out_dir)
    return out
