#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
if ! pkg-config --exists opencv4; then
    echo 'OpenCV development files missing: install libopencv-dev on Ubuntu/Orange Pi.' >&2
    exit 1
fi
TARGET="${1:-vision}"
case "$TARGET" in
    vision|main) SRC=main.cpp ;;
    test_frontend) SRC=test_frontend.cpp ;;
    test_gap) SRC=test_gap.cpp ;;
    test_obstacles) SRC=test_obstacles.cpp ;;
    test_obstacle_task) SRC=test_obstacle_task.cpp ;;
    test_vision_link) SRC=test_vision_link.cpp ;;
    test_sudoku) SRC=test_sudoku.cpp ;;
    test_run) SRC=test_run.cpp ;;
    *) echo 'Usage: bash build.sh [vision|main|test_frontend|test_gap|test_obstacles|test_obstacle_task|test_vision_link|test_sudoku|test_run]' >&2; exit 1 ;;
esac
read -r -a OPENCV_FLAGS <<< "$(pkg-config --cflags --libs opencv4)"
g++ -std=c++17 -O2 -Wall -Wextra "$SRC" -o "$TARGET" "${OPENCV_FLAGS[@]}" -pthread
echo "Built: $TARGET"
