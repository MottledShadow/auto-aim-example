#include "hik_camera.hpp"

namespace auto_aim
{

int initializeHikSdk()
{
    return MV_CC_Initialize();
}

int finalizeHikSdk()
{
    return MV_CC_Finalize();
}

int enumerateHikCameras(MV_CC_DEVICE_INFO_LIST& deviceList)
{
    deviceList = {};
    return MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
}

int createFirstHikCamera(void*& handle, const MV_CC_DEVICE_INFO_LIST& deviceList)
{
    if (deviceList.nDeviceNum == 0 || deviceList.pDeviceInfo[0] == nullptr)
    {
        return MV_E_PARAMETER;
    }

    return MV_CC_CreateHandle(&handle, deviceList.pDeviceInfo[0]);
}

int openHikCamera(void* handle)
{
    return MV_CC_OpenDevice(handle);
}

} // namespace auto_aim
