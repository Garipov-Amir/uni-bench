#!/usr/bin/env bash
# Download the public datasets of the instance families IF1-IF4 into ./datasets.
#   IF1 AreaCoverage (VM25, AC300, H2)        github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset
#   IF2 Fields2Benchmark, 350 fields (WKT)     doi:10.5281/zenodo.14524734 (CC BY 4.0)
#   IF3 indoor rooms (Bormann et al.)          github.com/ipa320/ipa_coverage_planning (test_maps)
#   IF4 SafeLog maps, 21 graphs x 400 vertices doi:10.5281/zenodo.4439404 (CC BY 4.0)
#   published MCPP instances of Tang & Ma (2023)  github.com/reso1/MIP-MCPP (validation, R9)
# IF5 is generated (`uninbench generate synthetic`).
set -euo pipefail
mkdir -p datasets && cd datasets
[ -d AreaCoverage-dataset ] || git clone --depth 1 https://github.com/UNCCharlotte-CS-Robotics/AreaCoverage-dataset.git
if [ ! -d test_maps ]; then
  git clone --depth 1 --filter=blob:none --sparse https://github.com/ipa320/ipa_coverage_planning.git ipa_tmp
  (cd ipa_tmp && git sparse-checkout set ipa_room_segmentation/common/files/test_maps)
  mv ipa_tmp/ipa_room_segmentation/common/files/test_maps test_maps && rm -rf ipa_tmp
fi
if [ ! -d fields2benchmark ]; then
  mkdir -p fields2benchmark
  curl -sSL -o fields2benchmark/wkt.zip https://zenodo.org/records/14524735/files/wkt.zip
  curl -sSL -o fields2benchmark/README.md https://zenodo.org/records/14524735/files/README.md
  (cd fields2benchmark && unzip -q wkt.zip && rm wkt.zip)
fi
if [ ! -d safelog ]; then
  mkdir -p safelog
  curl -sSL -o safelog/benchmark-planning.zip https://zenodo.org/records/4439404/files/benchmark-planning.zip
  (cd safelog && unzip -q benchmark-planning.zip && rm benchmark-planning.zip)
fi
# instances and published solutions of Tang & Ma (2023), used to validate MFC, MSTC*, MIP and LS-MCPP
# against published results (only the data is used; the repository's code is not part of uninbench)
[ -d MIP-MCPP ] || git clone --depth 1 https://github.com/reso1/MIP-MCPP.git
echo "datasets ready in $(pwd):"
echo "  uninbench import areacoverage datasets/AreaCoverage-dataset --out data/IF1 --stratify holes"
echo "  uninbench import fields datasets/fields2benchmark/wkt --out data/IF2 --stratify holes"
echo "  uninbench import rooms datasets/test_maps --out data/IF3"
echo "  uninbench import safelog datasets/safelog --out data/IF4 --stratify graph_density"
