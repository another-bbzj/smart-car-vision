#include "Car_Race.h"
#include "Car_Core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* A 200 Hz differential-drive plant with 25 Hz independent camera updates.
 * Generated course elements exercise real wheel targets and encoder feedback. */
static void two_laps(void) {
    CarRace r;
    CarVisionObservation o = {0};
    RaceDrive wheels = {{0,0}};
    unsigned ticks, gaps=0, stops=0, bypasses=0;
    unsigned stop_ms=0;
    float dl=0, dr=0, clearance=0;
    Race_Start(&r);
    for (ticks=0; ticks<160000 && r.phase!=RACE_FINISHED; ++ticks) {
        float position = fmodf(r.distance,RACE_LAP_M);
        RacePhase before = r.phase;
        if (ticks%8==0) {
            ++o.sequence;
            o.confidence=0.9f;
            o.element=VISION_ELEMENT_NONE;
            o.valid_mask=VISION_CURVATURE_VALID;
            o.curvature_inv_m=0;
            if (position>=5 && position<6.60f) { o.valid_mask=0; o.element=VISION_ELEMENT_GAP; }
            if (position>=12 && position<12.6f) {
                o.element=VISION_ELEMENT_YELLOW; o.distance_m=12.6f-position;
                o.valid_mask |= VISION_DISTANCE_VALID;
            }
            if (position>=20 && position<20.5f) {
                o.element=VISION_ELEMENT_RED; o.distance_m=20.5f-position;
                o.valid_mask |= VISION_DISTANCE_VALID;
            }
            if (position>=30 && position<31) o.element=VISION_ELEMENT_GREEN;
        }
        wheels=Race_Step(&r,&o,NULL,dl,dr,0.005f,0,0,dl==0 && dr==0,ticks*5);
        assert(r.phase!=RACE_FAULT);
        if (r.phase!=before && r.phase==RACE_GAP) ++gaps;
        if (r.phase!=before && r.phase==RACE_BYPASS) ++bypasses;
        if (r.phase==RACE_BYPASS && fabsf(r.goal_m-0.42f)<0.005f) clearance=fabsf(r.offset);
        if (r.phase!=before && r.phase==RACE_RED) { ++stops; stop_ms=ticks*5; }
        if (before==RACE_RED && r.phase!=before) assert(ticks*5-stop_ms>=5000);
        dl=wheels.rpm[0]*0.005f*6.2831853f*RACE_RADIUS_M/60;
        dr=wheels.rpm[1]*0.005f*6.2831853f*RACE_RADIUS_M/60;
    }
    assert(r.phase==RACE_FINISHED && gaps==2 && bypasses==2 && stops==1);
    assert(clearance>0.12f && wheels.rpm[0]==0 && wheels.rpm[1]==0);
}

int main(void) {
    CarRace r;
    RaceDrive cmd;
    Car_Core car;
    CarVisionObservation o = {0};
    uint8_t tiles[4][9]={{1,0,3,0,5,0,2,0,1},{4,0,0,0,2,3,5,0,4},
                         {5,6,0,0,1,2,6,0,0},{0,3,1,6,0,5,0,1,0}};
    const uint8_t solution[36]={1,2,3,4,5,6,4,5,6,1,2,3,2,3,1,5,6,4,
                                5,6,4,2,3,1,3,1,2,6,4,5,6,4,5,3,1,2};
    uint8_t answer[36];
    assert(Race_Solve(tiles,answer) && !memcmp(answer,solution,36));
    tiles[0][1]=1;
    assert(!Race_Solve(tiles,answer));
    memset(tiles,0,sizeof(tiles));
    assert(!Race_Solve(tiles,answer));
    Race_Start(&r);
    o.valid_mask=VISION_CURVATURE_VALID; o.curvature_inv_m=3; o.confidence=1;
    cmd=Race_Step(&r,&o,NULL,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[1]>cmd.rpm[0] && cmd.rpm[0]>0);
    Car_Init(&car);
    assert(Car_StartRace(&car));
    Car_RaceWheels(&car,cmd.rpm);
    Car_Tick(&car,0,0);
    assert(car.reference[0]>0 && car.reference[1]>car.reference[0]);
    Car_LocalStop(&car);
    assert(car.state==CAR_STOP && !car.autonomous && !car.output[1]);
    cmd=Race_Step(&r,NULL,NULL,0,0,0.005f,0,0,1,5);
    assert(r.phase==RACE_FAULT && cmd.rpm[0]==0);
    two_laps();
    puts("PASS race authority, geometry, fresh link, two laps, 1.60 m gaps, bypass clearance, red dwell, finish and Sudoku");
    return 0;
}
