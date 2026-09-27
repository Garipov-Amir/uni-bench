#!/usr/bin/env bash
# Download the public datasets used by the instance families (IF1-IF3) into ./datasets.
# IF4 (SafeLog graphs) is not publicly mirrored here; `uninbench generate safelog` creates a
# SafeLog-like connectivity sweep, and `uninbench import safelog <dir>` reads edge lists.
set -euo pipefail
mkdir -p datasets && cd datasets
[ -d AreaCoverage-dataset ] || git clone --depth 1 https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset.git
if [ ! -d test_maps ]; then
  git clone --depth 1 --filter=blob:none --sparse https://github.com/ipa320/ipa_coverage_planning.git ipa_tmp
  (cd ipa_tmp && git sparse-checkout set ipa_room_segmentation/common/files/test_maps)
  mv ipa_tmp/ipa_room_segmentation/common/files/test_maps test_maps && rm -rf ipa_tmp
fi
# Fields: place Fields2Benchmark / Fields2Cover field files (GML/XML, GeoJSON or WKT) in datasets/fields.
mkdir -p fields
for f in test1.xml test.json ee_field_130.wkt; do
  [ -f fields/$f ] || curl -sL -o fields/$f https://raw.githubusercontent.com/Fields2Cover/Fields2Cover/main/data/$f
done
echo "datasets ready in $(pwd)"
