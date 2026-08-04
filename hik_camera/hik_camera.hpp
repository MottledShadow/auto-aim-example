#pragma once

#include "MvCameraControl.h"

namespace auto_aim
{

int initializeHikSdk();
int finalizeHikSdk();
int enumerateHikCameras(MV_CC_DEVICE_INFO_LIST& deviceList);
int createFirstHikCamera(void*& handle, const MV_CC_DEVICE_INFO_LIST& deviceList);
int openHikCamera(void* handle);

} // namespace auto_aim
