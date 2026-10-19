#!/bin/sh
# Pruebas de lógica en PC (no sustituyen las pruebas en el robot). Uso: sh test/host/run.sh
set -e
D=$(cd "$(dirname "$0")" && pwd); B=$(mktemp -d)
CXX="${CXX:-g++} -std=c++17 -Wall -Wextra -O1"
$CXX -DMAZE_ROWS=3 -DMAZE_COLS=3 "$D/test_floodfill.cpp" -o "$B/ff33"; "$B/ff33"
$CXX -DMAZE_ROWS=3 -DMAZE_COLS=6 "$D/test_floodfill.cpp" -o "$B/ff36"; "$B/ff36"
$CXX "$D/test_navgeom.cpp" -o "$B/geom"; "$B/geom"
$CXX -I"$D/fake" "$D/test_leds.cpp" -o "$B/leds"; "$B/leds"
rm -rf "$B"
