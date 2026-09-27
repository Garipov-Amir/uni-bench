import os
import tempfile

import pytest

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "uninbench-mpl"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "uninbench-cache"))

from shapely.geometry import box  # noqa: E402

from uninbench.instance.model import default_robot, instance_from_polygon  # noqa: E402


@pytest.fixture
def rect_instance():
    return instance_from_polygon("rect", box(0, 0, 10, 4), default_robot(start=(0.5, 0.5, 0.0)))


@pytest.fixture
def room_with_hole():
    return instance_from_polygon("hole", box(0, 0, 12, 9).difference(box(4, 3, 8, 6)),
                                 default_robot(start=(0.5, 0.5, 0.0)))
