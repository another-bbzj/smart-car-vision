#include "Car_Race.h"
#include "Car_Core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* A 200 Hz differential-drive plant with 25 Hz independent camera updates.
 * Generated course elements exercise real wheel targets and encoder feedback. */
static void two_laps_rpm(float target_rpm) {
    CarRace r;
    CarVisionObservation o = {0};
    RaceDrive wheels = {{0,0}};
    unsigned ticks, gaps=0, stops=0, bypasses=0;
    unsigned stop_ms=0;
    float dl=0, dr=0, clearance=0, peak_center_rpm=0;
    if (target_rpm > 0.0f) Race_StartWithRpm(&r, target_rpm);
    else Race_Start(&r);
    for (ticks=0; ticks<240000 && r.phase!=RACE_FINISHED; ++ticks) {
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
        wheels=Race_Step(&r,&o,dl,dr,0.005f,0,0,dl==0 && dr==0,ticks*5);
        assert(r.phase!=RACE_FAULT);
        peak_center_rpm = fmaxf(peak_center_rpm, 0.5f * (wheels.rpm[0] + wheels.rpm[1]));
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
    if (target_rpm > 0.0f) {
        assert(peak_center_rpm <= target_rpm + 0.5f);
        assert(peak_center_rpm >= target_rpm - 1.0f);
    }
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
    assert(Race_ClampRpm(NAN) == (float)RACE_DEFAULT_RPM);
    assert(Race_ClampRpm(5.0f) == (float)RACE_MIN_RPM);
    assert(Race_ClampRpm(2000.0f) == (float)RACE_MAX_RPM);

    Race_Start(&r);
    o.valid_mask=VISION_CURVATURE_VALID; o.curvature_inv_m=3; o.confidence=1;
    cmd=Race_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[1]>cmd.rpm[0] && cmd.rpm[0]>0);
    Car_Init(&car);
    assert(Car_StartRace(&car));
    Car_RaceWheels(&car,cmd.rpm);
    Car_Tick(&car,0,0);
    assert(car.reference[0]>0 && car.reference[1]>car.reference[0]);
    Car_LocalStop(&car);
    assert(car.state==CAR_STOP && !car.autonomous && !car.output[1]);
    /* A curvature spike beyond the chassis turning limit must not command the
     * inner wheel to reverse while the car is moving forward: it may stop, but
     * its target stays on the commanded side instead of being lost. */
    o.curvature_inv_m=15;
    cmd=Race_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[0]>=0 && cmd.rpm[1]>0);
    o.curvature_inv_m=-15;
    cmd=Race_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[0]>0 && cmd.rpm[1]>=0);
    /* A missing/stale link must hold the last curvature and keep driving. */
    cmd=Race_Step(&r,NULL,0,0,0.005f,0,0,1,5);
    assert(r.phase==RACE_TRACK && r.degraded && cmd.rpm[0]>0 && cmd.rpm[1]>=0);
    /* Even a prolonged outage must not fault the race; only the warning stays. */
    for (unsigned i=0;i<400;++i) {
        cmd=Race_Step(&r,NULL,0,0,0.005f,0,0,1,20U+i*5U);
        assert(r.phase==RACE_TRACK && r.degraded);
    }
    /* A fresh frame that found no road is the same hold, not a fault. */
    o.valid_mask=0;
    ++o.sequence;
    cmd=Race_Step(&r,&o,0,0,0.005f,0,0,1,2500);
    assert(r.phase==RACE_TRACK && r.degraded && cmd.rpm[0]>0);
    /* The next valid curvature frame clears the warning. */
    o.valid_mask=VISION_CURVATURE_VALID;
    o.curvature_inv_m=0;
    o.confidence=1;
    ++o.sequence;
    cmd=Race_Step(&r,&o,0,0,0.005f,0,0,1,2505);
    assert(r.phase==RACE_TRACK && !r.degraded && cmd.rpm[0]>0);

    /* Custom target RPM: straight-line & blind phases respect configured RPM ceiling. */
    Race_StartWithRpm(&r, 40.0f);
    o.valid_mask = VISION_CURVATURE_VALID; o.curvature_inv_m = 0; o.confidence = 1;
    o.element = VISION_ELEMENT_NONE; ++o.sequence;
    cmd = Race_Step(&r, &o, 0, 0, 0.005f, 0, 0, 1, 10);
    assert(fabsf(cmd.rpm[0] - 40.0f) < 0.1f && fabsf(cmd.rpm[1] - 40.0f) < 0.1f);
    /* Gap phase at 40 rpm must not exceed 40 rpm. */
    r.straight_m = 0.2f;
    o.valid_mask = 0; o.element = VISION_ELEMENT_GAP; ++o.sequence;
    cmd = Race_Step(&r, &o, 0, 0, 0.005f, 0, 0, 0, 15);
    assert(r.phase == RACE_GAP && cmd.rpm[0] <= 40.1f && cmd.rpm[1] <= 40.1f);
    /* High RPM (240 rpm) reaches 240 rpm on straight. */
    Race_StartWithRpm(&r, 240.0f);
    o.valid_mask = VISION_CURVATURE_VALID; o.curvature_inv_m = 0; o.confidence = 1;
    o.element = VISION_ELEMENT_NONE; ++o.sequence;
    cmd = Race_Step(&r, &o, 0, 0, 0.005f, 0, 0, 1, 20);
    assert(fabsf(cmd.rpm[0] - 240.0f) < 0.2f && fabsf(cmd.rpm[1] - 240.0f) < 0.2f);

    two_laps_rpm(0.0f);
    two_laps_rpm(80.0f);
    two_laps_rpm(240.0f);
    puts("PASS race authority, configurable RPM, geometry, vision hold, two laps, gaps, bypass, red dwell, finish and Sudoku");
    return 0;
}
