#ifndef QUATIMU_H
#define QUATIMU_H

namespace FC {
void QuatIMU_Start(void);

void QuatIMU_Update( int * packetAddr );

void QuatIMU_CompassInitCalibrate(void);
void QuatIMU_CompassCalibrateAddSample(void);
void QuatIMU_CompassCalibrateComputeOffsets(void);
};

#endif // QUATIMU_H
