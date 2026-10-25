"""Instance families IF1-IF5 (Table: instance families of the benchmark)."""

from .areacoverage import import_areacoverage
from .fields import import_fields
from .rooms import import_rooms
from .safelog import import_safelog, synthetic_safelog
from .synthetic import generate, generate_family, regenerate, scale_family

FAMILIES = {
    "IF1": {"name": "polygonal environments", "source": "AreaCoverage: VM25, AC300, H2 (Agarwal & Akella, 2022)",
            "varied": "geometry; 1-15 holes (AC300)", "importer": "areacoverage",
            "url": "https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset"},
    "IF2": {"name": "agricultural fields", "source": "Fields2Benchmark (Mier Munoz et al., 2024)",
            "varied": "real non-convex boundaries, internal obstacles", "importer": "fields",
            "url": "https://github.com/Fields2Cover/Fields2Cover (field file formats)"},
    "IF3": {"name": "indoor rooms", "source": "room geometries of Bormann et al. (2018)",
            "varied": "without / with furniture (paired)", "importer": "rooms",
            "url": "https://github.com/ipa320/ipa_coverage_planning (ipa_room_segmentation/common/files/test_maps)"},
    "IF4": {"name": "connectivity sweep", "source": "SafeLog graphs (Rybecky & Kulich, 2021)",
            "varied": "graph density", "importer": "safelog", "url": None},
    "IF5": {"name": "synthetic", "source": "seeded generator", "varied": "size, obstacle density and shape, "
            "non-convexity", "importer": "synthetic", "url": None},
}

__all__ = ["FAMILIES", "generate", "generate_family", "regenerate", "scale_family", "import_areacoverage",
           "import_fields", "import_rooms", "import_safelog", "synthetic_safelog"]
