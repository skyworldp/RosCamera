#include "HikCamera.h"
#include "PixelType.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

namespace
{
size_t EstimateImageBufferSize(unsigned int pixelFormat, unsigned int width, unsigned int height)
{
    switch (pixelFormat)
    {
    case PixelType_Gvsp_Mono8:
        return static_cast<size_t>(width) * height;
    case PixelType_Gvsp_RGB8_Packed:
    case PixelType_Gvsp_BGR8_Packed:
    case PixelType_Gvsp_YUV444_Packed:
        return static_cast<size_t>(width) * height * 3;
    case PixelType_Gvsp_RGBA8_Packed:
    case PixelType_Gvsp_BGRA8_Packed:
        return static_cast<size_t>(width) * height * 4;
    default: {
        unsigned int bits =
            (pixelFormat & MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_MASK) >> MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_SHIFT;
        if (bits == 0)
        {
            bits = 24; // 默认按 24 bits 估算
        }
        unsigned int bytes = (bits + 7) / 8;
        return static_cast<size_t>(width) * height * bytes;
    }
    }
}
} // namespace

namespace hik
{

// 构造函数
HikCamera::HikCamera()
    : m_handle(nullptr), m_isOpen(false), m_isGrabbing(false), m_convertBuffer(nullptr), m_bufferSize(0),
      m_deviceIndex(0), m_savedExposure(0.0f), m_savedGain(0.0f), m_savedTrigger(false), m_savedFrameRate(0.0f),
      m_savedPixelFormat(0)
{
}

// 设置帧率 (fps)
bool HikCamera::SetFrameRate(float fps)
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    if (fps <= 0.0f)
    {
        m_lastError = "Frame rate must be positive";
        return false;
    }

    // 尝试开启帧率控制开关
    int ret = MV_CC_SetBoolValue(m_handle, "AcquisitionFrameRateEnable", true);
    if (ret != MV_OK)
    {
        // 某些机型使用枚举形式
        ret = MV_CC_SetEnumValue(m_handle, "AcquisitionFrameRateEnable", 1);
        if (ret != MV_OK)
        {
            std::cerr << "HikCamera Warning: enable AcquisitionFrameRate failed (error code: 0x" << std::hex << ret
                      << std::dec << ")" << std::endl;
        }
    }

    // 关闭自动帧率（如果支持）
    ret = MV_CC_SetEnumValue(m_handle, "AcquisitionFrameRateAuto", 0);
    if (ret != MV_OK)
    {
        // 某些型号不支持该属性，忽略即可
    }

    ret = MV_CC_SetFloatValue(m_handle, "AcquisitionFrameRate", fps);
    if (ret != MV_OK)
    {
        // 如果设置失败，查询限制后再夹取
        MVCC_FLOATVALUE limits;
        if (MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &limits) == MV_OK)
        {
            float clamped = std::clamp(fps, limits.fMin, limits.fMax);
            if (std::fabs(clamped - fps) > 0.1f)
            {
                std::cerr << "HikCamera Warning: requested frame rate " << fps << " fps is outside camera limits ("
                          << limits.fMin << "-" << limits.fMax << "), clamped to " << clamped << " fps" << std::endl;
            }
            ret = MV_CC_SetFloatValue(m_handle, "AcquisitionFrameRate", clamped);
        }

        if (ret != MV_OK)
        {
            SetError("Set frame rate failed", ret);
            return false;
        }
    }

    MVCC_FLOATVALUE current;
    float applied = fps;
    ret = MV_CC_GetFloatValue(m_handle, "ResultingFrameRate", &current);
    if (ret == MV_OK)
    {
        applied = current.fCurValue;
    }
    else if (MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &current) == MV_OK)
    {
        applied = current.fCurValue;
    }

    if (std::fabs(applied - fps) > 0.1f)
    {
        std::cerr << "HikCamera Warning: requested frame rate " << fps << " fps but camera applied " << applied
                  << " fps" << std::endl;
    }

    m_savedFrameRate = applied;
    return true;
}

// 获取帧率
float HikCamera::GetFrameRate()
{
    if (!m_isOpen)
    {
        return 0.0f;
    }

    MVCC_FLOATVALUE value;
    int ret = MV_CC_GetFloatValue(m_handle, "ResultingFrameRate", &value);
    if (ret == MV_OK)
    {
        return value.fCurValue;
    }

    ret = MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &value);
    if (ret == MV_OK)
    {
        return value.fCurValue;
    }

    return 0.0f;
}

// 设置像素格式（通过像素格式常量）
bool HikCamera::SetPixelFormat(unsigned int pixelFormat)
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    if (m_savedPixelFormat == pixelFormat && pixelFormat != 0)
    {
        return true;
    }

    bool resumeGrabbing = m_isGrabbing;
    if (resumeGrabbing)
    {
        if (!StopGrabbing())
        {
            return false;
        }
    }

    int ret = MV_CC_SetEnumValue(m_handle, "PixelFormat", pixelFormat);
    if (ret != MV_OK)
    {
        SetError("Set pixel format failed", ret);
        if (resumeGrabbing)
        {
            // 尝试恢复采集，避免外部状态改变
            StartGrabbing();
        }
        return false;
    }

    m_savedPixelFormat = pixelFormat;

    if (resumeGrabbing)
    {
        if (!StartGrabbing())
        {
            return false;
        }
    }

    return true;
}

// 获取像素格式
unsigned int HikCamera::GetPixelFormat()
{
    if (!m_isOpen)
    {
        return 0;
    }

    MVCC_ENUMVALUE value;
    int ret = MV_CC_GetEnumValue(m_handle, "PixelFormat", &value);
    if (ret != MV_OK)
    {
        return 0;
    }

    return value.nCurValue;
}

// 析构函数
HikCamera::~HikCamera()
{
    if (m_isGrabbing)
    {
        StopGrabbing();
    }
    if (m_isOpen)
    {
        Close();
    }
    if (m_convertBuffer)
    {
        delete[] m_convertBuffer;
        m_convertBuffer = nullptr;
    }
}

// 设置错误信息
void HikCamera::SetError(const std::string &error, int errorCode)
{
    std::ostringstream oss;
    oss << error << " (Error code: 0x" << std::hex << errorCode << ")";
    m_lastError = oss.str();
    std::cerr << "HikCamera Error: " << m_lastError << std::endl;
}

// 枚举所有可用相机
std::vector<CameraInfo> HikCamera::EnumerateDevices()
{
    std::vector<CameraInfo> devices;

    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (ret != MV_OK)
    {
        std::cerr << "Enumerate devices failed! Error code: 0x" << std::hex << ret << std::endl;
        return devices;
    }

    std::cout << "Found " << deviceList.nDeviceNum << " device(s)" << std::endl;

    // 遍历设备列表
    for (unsigned int i = 0; i < deviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO *pDeviceInfo = deviceList.pDeviceInfo[i];
        if (!pDeviceInfo)
            continue;

        CameraInfo info;
        info.deviceType = pDeviceInfo->nTLayerType;

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            // GigE 相机
            MV_GIGE_DEVICE_INFO *pGigEInfo = &pDeviceInfo->SpecialInfo.stGigEInfo;

            info.serialNumber = std::string((char *)pGigEInfo->chSerialNumber);
            info.modelName = std::string((char *)pGigEInfo->chModelName);

            // 格式化 IP 地址
            unsigned int ip = pGigEInfo->nCurrentIp;
            std::ostringstream oss;
            oss << ((ip & 0xFF000000) >> 24) << "." << ((ip & 0x00FF0000) >> 16) << "." << ((ip & 0x0000FF00) >> 8)
                << "." << (ip & 0x000000FF);
            info.ipAddress = oss.str();

            std::cout << "[" << i << "] GigE Camera:" << std::endl;
            std::cout << "    Model: " << info.modelName << std::endl;
            std::cout << "    Serial: " << info.serialNumber << std::endl;
            std::cout << "    IP: " << info.ipAddress << std::endl;
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            // USB 相机
            MV_USB3_DEVICE_INFO *pUSBInfo = &pDeviceInfo->SpecialInfo.stUsb3VInfo;

            info.serialNumber = std::string((char *)pUSBInfo->chSerialNumber);
            info.modelName = std::string((char *)pUSBInfo->chModelName);
            info.ipAddress = "USB";

            std::cout << "[" << i << "] USB Camera:" << std::endl;
            std::cout << "    Model: " << info.modelName << std::endl;
            std::cout << "    Serial: " << info.serialNumber << std::endl;
        }

        devices.push_back(info);
    }

    return devices;
}

// 打开相机 (通过索引)
bool HikCamera::Open(unsigned int index)
{
    if (m_isOpen)
    {
        m_lastError = "Camera is already open";
        return false;
    }

    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (ret != MV_OK)
    {
        SetError("Enumerate devices failed", ret);
        return false;
    }

    if (index >= deviceList.nDeviceNum)
    {
        m_lastError = "Invalid device index";
        return false;
    }

    // 创建句柄
    ret = MV_CC_CreateHandle(&m_handle, deviceList.pDeviceInfo[index]);
    if (ret != MV_OK)
    {
        SetError("Create handle failed", ret);
        return false;
    }

    // 打开设备
    ret = MV_CC_OpenDevice(m_handle);
    if (ret != MV_OK)
    {
        SetError("Open device failed", ret);
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
        return false;
    }

    // 设置触发模式为关闭（连续采集）
    ret = MV_CC_SetEnumValue(m_handle, "TriggerMode", 0);
    if (ret != MV_OK)
    {
        std::cerr << "Warning: Set trigger mode failed, error code: 0x" << std::hex << ret << std::endl;
    }

    m_isOpen = true;
    std::cout << "Camera opened successfully (index: " << index << ")" << std::endl;
    return true;
}

// 打开相机 (通过序列号)
bool HikCamera::OpenBySerialNumber(const std::string &serialNumber)
{
    if (m_isOpen)
    {
        m_lastError = "Camera is already open";
        return false;
    }

    MV_CC_DEVICE_INFO_LIST deviceList;
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (ret != MV_OK)
    {
        SetError("Enumerate devices failed", ret);
        return false;
    }

    // 查找指定序列号的设备
    int deviceIndex = -1;
    for (unsigned int i = 0; i < deviceList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO *pDeviceInfo = deviceList.pDeviceInfo[i];
        std::string sn;

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            sn = std::string((char *)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            sn = std::string((char *)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }

        if (sn == serialNumber)
        {
            deviceIndex = i;
            break;
        }
    }

    if (deviceIndex < 0)
    {
        m_lastError = "Device with serial number '" + serialNumber + "' not found";
        return false;
    }

    return Open(deviceIndex);
}

// 关闭相机
bool HikCamera::Close()
{
    if (!m_isOpen)
    {
        return true;
    }

    if (m_isGrabbing)
    {
        StopGrabbing();
    }

    // 关闭设备
    int ret = MV_CC_CloseDevice(m_handle);
    if (ret != MV_OK)
    {
        SetError("Close device failed", ret);
    }

    // 销毁句柄
    ret = MV_CC_DestroyHandle(m_handle);
    if (ret != MV_OK)
    {
        SetError("Destroy handle failed", ret);
    }

    m_handle = nullptr;
    m_isOpen = false;
    std::cout << "Camera closed" << std::endl;
    return true;
}

// 开始采集
bool HikCamera::StartGrabbing()
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    if (m_isGrabbing)
    {
        return true;
    }

    int ret = MV_CC_StartGrabbing(m_handle);
    if (ret != MV_OK)
    {
        SetError("Start grabbing failed", ret);
        return false;
    }

    m_isGrabbing = true;
    std::cout << "Start grabbing" << std::endl;
    return true;
}

// 停止采集
bool HikCamera::StopGrabbing()
{
    if (!m_isGrabbing)
    {
        return true;
    }

    int ret = MV_CC_StopGrabbing(m_handle);
    if (ret != MV_OK)
    {
        SetError("Stop grabbing failed", ret);
        return false;
    }

    m_isGrabbing = false;
    std::cout << "Stop grabbing" << std::endl;
    return true;
}

// 获取一帧图像（可选目标像素格式）
bool HikCamera::GrabImage(ImageData &imageData, unsigned int timeout, unsigned int desiredPixelFormat)
{
    if (!m_isGrabbing)
    {
        m_lastError = "Camera is not grabbing";
        return false;
    }

    MV_FRAME_OUT frameInfo;
    memset(&frameInfo, 0, sizeof(MV_FRAME_OUT));

    int ret = MV_CC_GetImageBuffer(m_handle, &frameInfo, timeout);
    if (ret != MV_OK)
    {
        if (ret != static_cast<int>(MV_E_NODATA))
        {
            SetError("Get image buffer failed", ret);
        }
        return false;
    }

    const unsigned int width = frameInfo.stFrameInfo.nWidth;
    const unsigned int height = frameInfo.stFrameInfo.nHeight;
    const unsigned int srcPixelFormat = frameInfo.stFrameInfo.enPixelType;
    const unsigned int targetPixelFormat = desiredPixelFormat != 0 ? desiredPixelFormat : srcPixelFormat;
    const bool needConvert = (targetPixelFormat != srcPixelFormat);

    size_t requiredSize = needConvert ? EstimateImageBufferSize(targetPixelFormat, width, height)
                                      : static_cast<size_t>(frameInfo.stFrameInfo.nFrameLen);
    if (requiredSize == 0)
    {
        requiredSize = static_cast<size_t>(frameInfo.stFrameInfo.nFrameLen);
    }

    if (m_bufferSize < requiredSize)
    {
        if (m_convertBuffer)
        {
            delete[] m_convertBuffer;
        }
        m_convertBuffer = new unsigned char[requiredSize];
        m_bufferSize = requiredSize;
    }

    bool conversionDone = false;
    bool success = true;

    if (needConvert)
    {
        MV_CC_PIXEL_CONVERT_PARAM convertParam;
        memset(&convertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
        convertParam.nWidth = width;
        convertParam.nHeight = height;
        convertParam.pSrcData = frameInfo.pBufAddr;
        convertParam.nSrcDataLen = frameInfo.stFrameInfo.nFrameLen;
        convertParam.enSrcPixelType = static_cast<MvGvspPixelType>(srcPixelFormat);
        convertParam.enDstPixelType = static_cast<MvGvspPixelType>(targetPixelFormat);
        convertParam.pDstBuffer = m_convertBuffer;
        convertParam.nDstBufferSize = static_cast<unsigned int>(m_bufferSize);

        int convertRet = MV_CC_ConvertPixelType(m_handle, &convertParam);
        if (convertRet == MV_OK)
        {
            conversionDone = true;
            imageData.pixelFormat = targetPixelFormat;
            imageData.data = m_convertBuffer;
            imageData.dataSize = requiredSize;
        }
        else
        {
            success = false;
            SetError("Convert pixel type failed", convertRet);
        }
    }

    if (!conversionDone)
    {
        size_t copySize = static_cast<size_t>(frameInfo.stFrameInfo.nFrameLen);
        if (m_bufferSize < copySize)
        {
            delete[] m_convertBuffer;
            m_convertBuffer = new unsigned char[copySize];
            m_bufferSize = copySize;
        }
        std::memcpy(m_convertBuffer, frameInfo.pBufAddr, copySize);
        imageData.pixelFormat = srcPixelFormat;
        imageData.data = m_convertBuffer;
        imageData.dataSize = copySize;
        // 如果原始数据被返回给调用者（未转换），仍然认为本次抓取成功
        success = success && !needConvert;
    }

    imageData.width = width;
    imageData.height = height;

    MV_CC_FreeImageBuffer(m_handle, &frameInfo);

    return success;
}

// 设置曝光时间
bool HikCamera::SetExposureTime(float exposureTime)
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    int ret = MV_CC_SetFloatValue(m_handle, "ExposureTime", exposureTime);
    if (ret != MV_OK)
    {
        SetError("Set exposure time failed", ret);
        return false;
    }

    // 保存当前设置以便重连后恢复
    m_savedExposure = exposureTime;

    return true;
}

// 获取曝光时间
float HikCamera::GetExposureTime()
{
    if (!m_isOpen)
    {
        return 0.0f;
    }

    MVCC_FLOATVALUE value;
    int ret = MV_CC_GetFloatValue(m_handle, "ExposureTime", &value);
    if (ret != MV_OK)
    {
        return 0.0f;
    }

    return value.fCurValue;
}

// 设置增益
bool HikCamera::SetGain(float gain)
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    int ret = MV_CC_SetFloatValue(m_handle, "Gain", gain);
    if (ret != MV_OK)
    {
        SetError("Set gain failed", ret);
        return false;
    }

    // 保存当前设置以便重连后恢复
    m_savedGain = gain;

    return true;
}

// 获取增益
float HikCamera::GetGain()
{
    if (!m_isOpen)
    {
        return 0.0f;
    }

    MVCC_FLOATVALUE value;
    int ret = MV_CC_GetFloatValue(m_handle, "Gain", &value);
    if (ret != MV_OK)
    {
        return 0.0f;
    }

    return value.fCurValue;
}

// 设置触发模式
bool HikCamera::SetTriggerMode(bool enable)
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    int ret = MV_CC_SetEnumValue(m_handle, "TriggerMode", enable ? 1 : 0);
    if (ret != MV_OK)
    {
        SetError("Set trigger mode failed", ret);
        return false;
    }

    // 保存当前触发模式以便重连后恢复
    m_savedTrigger = enable;

    return true;
}

// 重连实现
bool HikCamera::Reconnect(unsigned int index, int maxRetries, int retryDelayMs)
{
    // 保存索引
    m_deviceIndex = index;

    for (int attempt = 1; attempt <= maxRetries && m_isOpen == false; ++attempt)
    {
        std::cout << "Attempting reconnect (index=" << index << ") try=" << attempt << "..." << std::endl;

        // 确保先关闭旧的资源
        if (m_isGrabbing)
        {
            StopGrabbing();
        }
        if (m_isOpen)
        {
            Close();
        }

        // 退避等待
        std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs * attempt));

        // 重新打开
        if (!Open(index))
        {
            std::cout << "Reconnect open failed: " << GetLastError() << std::endl;
            continue;
        }

        // 恢复设置
        if (m_savedExposure > 0.0f)
        {
            SetExposureTime(m_savedExposure);
        }
        if (m_savedGain > 0.0f)
        {
            SetGain(m_savedGain);
        }
        if (m_savedFrameRate > 0.0f)
        {
            SetFrameRate(m_savedFrameRate);
        }
        if (m_savedPixelFormat > 0)
        {
            SetPixelFormat(m_savedPixelFormat);
        }
        SetTriggerMode(m_savedTrigger);

        // 启动采集
        if (!StartGrabbing())
        {
            std::cout << "Reconnect start grabbing failed: " << GetLastError() << std::endl;
            Close();
            continue;
        }

        std::cout << "Reconnect successful" << std::endl;
        return true;
    }

    std::cout << "Reconnect failed after " << maxRetries << " attempts" << std::endl;
    return false;
}

// 软件触发一次
bool HikCamera::TriggerSoftware()
{
    if (!m_isOpen)
    {
        m_lastError = "Camera is not open";
        return false;
    }

    int ret = MV_CC_SetCommandValue(m_handle, "TriggerSoftware");
    if (ret != MV_OK)
    {
        SetError("Software trigger failed", ret);
        return false;
    }

    return true;
}

// 获取图像宽度
unsigned int HikCamera::GetWidth()
{
    if (!m_isOpen)
    {
        return 0;
    }

    MVCC_INTVALUE value;
    int ret = MV_CC_GetIntValue(m_handle, "Width", &value);
    if (ret != MV_OK)
    {
        return 0;
    }

    return value.nCurValue;
}

// 获取图像高度
unsigned int HikCamera::GetHeight()
{
    if (!m_isOpen)
    {
        return 0;
    }

    MVCC_INTVALUE value;
    int ret = MV_CC_GetIntValue(m_handle, "Height", &value);
    if (ret != MV_OK)
    {
        return 0;
    }

    return value.nCurValue;
}

} // namespace hik
