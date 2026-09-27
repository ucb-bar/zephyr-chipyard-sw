/* SPDX-License-Identifier: Apache-2.0
 * Startup calibration and parked bias tracking extracted from main.cpp.
 * GYRO_CAL_* and GBIAS_TRACK_* are supplied by the compiling firmware preset.
 */
#ifndef ROSE_FC_SHARED_GYRO_HPP
#define ROSE_FC_SHARED_GYRO_HPP
#include <math.h>
#include <stdint.h>
struct FcGyroCalibration {
    bool done=false;
    float bias[3]={};
    double sum[3]={};
    int n=0;
    int64_t since=0;
    void update(float gyro[3], bool armed, int64_t now_ms)
    {
        if (!done) {
            const bool still=fabsf(gyro[0])<GYRO_CAL_STILL_RADPS &&
                fabsf(gyro[1])<GYRO_CAL_STILL_RADPS && fabsf(gyro[2])<GYRO_CAL_STILL_RADPS;
            if (!still) { since=0; sum[0]=sum[1]=sum[2]=0.0; n=0; }
            else {
                if (!since) { since=now_ms; sum[0]=sum[1]=sum[2]=0.0; n=0; }
                sum[0]+=gyro[0]; sum[1]+=gyro[1]; sum[2]+=gyro[2]; ++n;
                if (now_ms-since>=(int64_t)(GYRO_CAL_SECONDS*1000.0f) && n>0) {
                    bias[0]=(float)(sum[0]/n); bias[1]=(float)(sum[1]/n); bias[2]=(float)(sum[2]/n);
                    done=true;
                }
            }
        }
        if (done && !armed && fabsf(gyro[0]-bias[0])<GBIAS_TRACK_STILL_RADPS &&
            fabsf(gyro[1]-bias[1])<GBIAS_TRACK_STILL_RADPS && fabsf(gyro[2]-bias[2])<GBIAS_TRACK_STILL_RADPS) {
            bias[0]+=GBIAS_TRACK_GAIN*(gyro[0]-bias[0]);
            bias[1]+=GBIAS_TRACK_GAIN*(gyro[1]-bias[1]);
            bias[2]+=GBIAS_TRACK_GAIN*(gyro[2]-bias[2]);
        }
        gyro[0]-=bias[0]; gyro[1]-=bias[1]; gyro[2]-=bias[2];
    }
};
#endif
