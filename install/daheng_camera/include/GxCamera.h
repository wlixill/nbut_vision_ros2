//--------------------------------------------------------------------------------
/**
\file     GxCamera.h
\brief    GxCamera class declare

\date     2020-05-14
\author   Qunshan He,mountain.he@qq.com

*/
//----------------------------------------------------------------------------------


#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "./daheng_sdk/include/GxIAPI.h"
#include "./daheng_sdk/include/DxImageProc.h"


#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block

void GetErrorString(GX_STATUS emErrorStatus);
void *ProcGetImage(void* pAcquisitionThread);

#define GX_VER(emStatus)\
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        std::cout<<"error"<<endl;          \
        break;                             \
    }
//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        exit(emStatus);                    \
    }

/// Roi and resolution Param
typedef struct Roi
{
    int64_t              m_i64Width;                ///< Image roi width
    int64_t              m_i64Height;               ///< Image roi height
    int64_t              m_i64OffsetX;              ///< OffsetX of roi
    int64_t              m_i64OffsetY;              ///< OffsetY of roi

    Roi()
    {
        //default roi is a 640*480 area in the center     默认的roi是640*480的中心区域 
        m_i64Width = 640;             ///< Roi width  640
        m_i64Height = 480;            ///< Roi height 480
        m_i64OffsetX = 80;            ///< OffsetX 80  和Min的差必须是16的倍数
        m_i64OffsetY = 60;            ///< OffsetY 60   和Min的差必须是2的倍数
    }
}Roi;

/// Exposure and Gain Param struct  曝光和图像通道增益
typedef struct ExposureGain
{

    bool                 m_bAutoExposure;               ///< Exposure is auto mode or not
    bool                 m_bAutoGain;                   ///< Gain is auto mode or not

    double               m_dExposureTime;               ///< Exposure Time
    double               m_dAutoExposureTimeMax;        ///< Maximum exposure time when using AutoExporsureTime mode
    double               m_dAutoExposureTimeMin;        ///< Minimum exposure time when using AutoExporsureTime mode

    double               m_dGain;                       ///< Gain
    double               m_dAutoGainMax;                ///< Maximum gain when using AutoGain mode
    double               m_dAutoGainMin;                ///< Minimum gain when using AutoGain mode

    int64_t              m_i64GrayValue;                ///< Expected gray value

    ///Default value
    ExposureGain()
    {
        m_bAutoExposure = true;              ///< Exposure is auto mode or not
        m_bAutoGain = true;                  ///< Gain is auto mode or not

        m_dExposureTime = 2000;               ///< 2000us Exposure Time
        m_dAutoExposureTimeMax = 10000;        ///< 5000us Maximum exposure time when using AutoExporsureTime mode
        m_dAutoExposureTimeMin = 500;        ///< 1000us Minimum exposure time when using AutoExporsureTime mode

        m_dGain = 6;                          ///< Gain (Maxium 16dB)
        m_dAutoGainMax = 10;                   ///< Maximum gain when using AutoGain mode
        m_dAutoGainMin = 5;                   ///< Minimum gain when using AutoGain mode

        m_i64GrayValue = 200;                 ///< Expected gray value
    }
}ExposureGain;

/// WhiteBalance
typedef struct WhiteBalance
{
    bool m_bWhiteBalance;                          ///< Auto WhiteBalance is applied ?
    GX_AWB_LAMP_HOUSE_ENTRY lightSource;           ///< The lamp type of environment
    WhiteBalance()
    {
        m_bWhiteBalance = false;                   ///< WhiteBalance is applied defaultly
        lightSource = GX_AWB_LAMP_HOUSE_ADAPTIVE;  ///< Auto adaptive mode
    }
}WhiteBalance;

class GxCamera
{
    //friend void *ProcGetImage(void* pMatImage);

public:
    GxCamera();
    ~GxCamera();

    ///initial
    // bool initial();
    bool initial(int exposure_time = 1500, float gain = 16.0);

    /// Is open
    bool isOpen();

    /// Close devise
    bool close(); 

    /// Get exposure time(us)
    void get_exposure_us(double* us);

    // Capture single picture
    bool read(cv::Mat* targetMatImg, float* timestamp);

    /// Open device and display device information
    bool openDevice(bool useSN, const char* CameraSN);

    /// Set trigger mode
    void setTriggerParam(bool useHardTrigger);

    /// Set camera exposure and gain
    void setExposureGainParam( bool AutoExposure,
                               bool AutoGain,
                               double ExposureTime,
                               double AutoExposureTimeMin,
                               double AutoExposureTimeMax,
                               double Gain,
                               double AutoGainMin,
                               double AutoGainMax,
                               int64_t GrayValue);

    /// Set camera roi param
    void setRoiParam( int64_t Width,
                      int64_t Height,
                      int64_t OffsetX,
                      int64_t OffsetY);

    /// Set camera WhiteBalance
    void setWhiteBalanceParam( bool WhiteBalanceOn,
                               GX_AWB_LAMP_HOUSE_ENTRY lightSource);

private:

    /// Set camera roi param
    GX_STATUS setRoi();

    /// Set camera exposure and gain
    GX_STATUS setExposureGain();

    /// Set camera WhiteBalance
    GX_STATUS setWhiteBalance();

    GX_STATUS setTrigger();

    GX_STATUS setBufferSize();

private:
    GX_DEV_HANDLE g_hDevice;                    ///< Device handle
    char* g_pDeviceSN;                          ///< Device SN number
    int64_t g_i64ColorFilter;                   ///< Color filter of device
    int64_t g_nPayloadSize = 0;                 ///< Payload size
    uint32_t ui32DeviceNum = 0;                 ///< device number

    bool g_bTriggerMode;                        ///< Hardware Trigger 1  flag
    bool g_bColorFilter;                        ///< Color filter support flag
    bool g_bExposure;                           ///< Exposure supprot flag
    bool g_bGain;                               ///< Gain supprot flag
    bool g_bImgImprovement;                     ///< Image improvement supprot flag
    bool g_bRoi;                                ///< Roi support flag
    bool g_bWhiteBalance;                       ///< WhiteBalance support flag

    ExposureGain exposure_gain;                 ///< Camera exposure and gain param
    Roi roi;                                    ///< Camera roi and resolution param
    WhiteBalance white_balance;                 ///< Camera whitebalance param
};

