#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
out="$root/build/race-check"
mkdir -p "$out"
cd "$root"
ccflags=(-std=c99 -Wall -Wextra -Werror -I CODE)
core=(CODE/Car_Core.c CODE/Car_PID.c CODE/Car_Reference.c)
cc "${ccflags[@]}" Tests/test_car_core.c "${core[@]}" -lm -o "$out/core"
cc "${ccflags[@]}" Tests/test_car_pid.c CODE/Car_PID.c CODE/Car_Reference.c -lm -o "$out/pid"
cc "${ccflags[@]}" Tests/test_car_vision.c CODE/Car_Vision.c -lm -o "$out/link"
cc "${ccflags[@]}" Tests/test_car_race.c CODE/Car_Race.c CODE/Car_Sudoku.c "${core[@]}" -lm -o "$out/race"
cc "${ccflags[@]}" Tests/test_car_menu.c CODE/Car_Menu.c "${core[@]}" -lm -o "$out/menu"
cc "${ccflags[@]}" Tests/test_car_params.c CODE/Car_Params.c "${core[@]}" -lm -o "$out/params"
cc "${ccflags[@]}" Tests/test_car_sudoku.c CODE/Car_Sudoku.c -lm -o "$out/sudoku_test"
for t in core pid link race menu params sudoku_test; do "$out/$t"; done
cd vision_inspection/vision
read -ra flags <<< "$(pkg-config --cflags --libs opencv4)"
build_one() {
    local extra=()
    if [[ "$1" == test_race_camera ]]; then
        extra=(-I "$root/CODE" "$out/receiver.o" "$out/mission.o" "$out/sudoku.o" -lutil)
    fi
    g++ -O2 -std=c++17 -Wall -Wextra "$1.cpp" -o "$out/$1" "${extra[@]}" "${flags[@]}" -pthread
}
cc -std=c99 -c "$root/CODE/Car_Vision.c" -o "$out/receiver.o"
cc -std=c99 -c "$root/CODE/Car_Race.c" -o "$out/mission.o"
cc -std=c99 -c "$root/CODE/Car_Sudoku.c" -o "$out/sudoku.o"
pids=()
for t in main test_frontend test_gap test_obstacles test_obstacle_task test_vision_link test_sudoku test_run test_race_camera; do
    build_one "$t" &
    pids+=("$!")
    if (( ${#pids[@]} == 2 )); then
        for pid in "${pids[@]}"; do wait "$pid"; done
        pids=()
    fi
done
for pid in "${pids[@]}"; do wait "$pid"; done
for t in test_frontend test_gap test_obstacles test_obstacle_task test_vision_link test_sudoku; do "$out/$t"; done
replay_root=$(mktemp -d "$out/sudoku-replay.XXXXXX")
"$out/test_race_camera" "$replay_root/tiles.avi"
"$out/main" --video "$replay_root/tiles.avi" --output "$replay_root/output" --save-every 1000
python3 - "$replay_root/output/sudoku.csv" <<'PY'
import csv
import sys
with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
expected = [
    [1,0,3,0,5,0,2,0,1], [4,0,0,0,2,3,5,0,4],
    [5,6,0,0,1,2,6,0,0], [0,3,1,6,0,5,0,1,0],
]
assert {int(row["tile"]) for row in rows} == {0,1,2,3}, rows
for row in rows:
    assert [int(row[f"c{i}"]) for i in range(9)] == expected[int(row["tile"])], row
    assert row["sent"] == "0"
print("PASS production vision entry point reads all four tiles and records exact givens")
PY
(cd "$out" && ./test_run "$root/vision_inspection/vision/test_fixtures/straight.png")
"$out/main" --video test_fixtures/sequence.avi
