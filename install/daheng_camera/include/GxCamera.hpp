//--------------------------------------------------------------------------------
/**
\file     GxCamera.h
\brief    GxCamera class declare

\date     2020-05-14
\author   Qunshan He,mountain.he@qq.com

*/
//----------------------------------------------------------------------------------
#pragma once
#ifndef RM_VERSION4_AUTOAIM_INCLUDE_GXCAMERA_HPP_
#define RM_VERSION4_AUTOAIM_INCLUDE_GXCAMERA_HPP_

#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "daheng_sdk/include/DxImageProc.h"
#include "daheng_sdk/include/GxIAPI.h"

#define ACQ_BUFFER_NUM 5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE (64 * 1024)  ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64     ///< Qty. of data transfer block

void GetErrorString(GX_STATUS emErrorStatus);
void * ProcGetImage(void * pAcquisitionThread);

#define GX_VER(emStatus)               \
  if (emStatus != GX_STATUS_SUCCESS) { \
    std::cout << "error" << endl;      \
    break;                             \
  }
// Show error message
#define GX_VERIFY(emStatus)            \
  if (emStatus != GX_STATUS_SUCCESS) { \
    GetErrorString(emStatus);          \
    return emStatus;                   \
  }

// Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus)       \
  if (emStatus != GX_STATUS_SUCCESS) { \
    GetErrorString(emStatus);          \
    GXCloseDevice(g_hDevice);          \
    g_hDevice = NULL;                  \
    GXCloseLib();                      \
    printf("<App Exit!>\n");           \
  }
// exit(emStatus);
/// Roi and resolution Param
typedef struct Roi
{
  int64_t m_i64Width;    ///< Image roi width
  int64_t m_i64Height;   ///< Image roi height
  int64_t m_i64OffsetX;  ///< OffsetX of roi
  int64_t m_i64OffsetY;  ///< OffsetY of roi

  Roi()
  {
    // default roi is a 640*480 area in the center     默认的roi是640*480的中心区域
    m_i64Width = 640;   ///< Roi width  640
    m_i64Height = 480;  ///< Roi height 480
    m_i64OffsetX = 80;  ///< OffsetX 80  和Min的差必须是16的倍数
    m_i64OffsetY = 60;  ///< OffsetY 60   和Min的差必须是2的倍数
  }
} Roi;

/// Exposure and Gain Param struct  曝光和图像通道增益
typedef struct ExposureGain
{
  bool m_bAutoExposure;  ///< Exposure is auto mode or not
  bool m_bAutoGain;      ///< Gain is auto mode or not

  double m_dExposureTime;         ///< Exposure Time
  double m_dAutoExposureTimeMax;  ///< Maximum exposure time when using AutoExporsureTime mode
  double m_dAutoExposureTimeMin;  ///< Minimum exposure time when using AutoExporsureTime mode

  double m_dGain;         ///< Gain
  double m_dAutoGainMax;  ///< Maximum gain when using AutoGain mode
  double m_dAutoGainMin;  ///< Minimum gain when using AutoGain mode

  int64_t m_i64GrayValue;  ///< Expected gray value

  /// Default value
  ExposureGain()
  {
    m_bAutoExposure = true;  ///< Exposure is auto mode or not
    m_bAutoGain = true;      ///< Gain is auto mode or not

    m_dExposureTime = 2000;  ///< 2000us Exposure Time
    m_dAutoExposureTimeMax =
      10000;  ///< 5000us Maximum exposure time when using AutoExporsureTime mode
    m_dAutoExposureTimeMin =
      500;  ///< 1000us Minimum exposure time when using AutoExporsureTime mode

    m_dGain = 6;          ///< Gain (Maxium 16dB)
    m_dAutoGainMax = 10;  ///< Maximum gain when using AutoGain mode
    m_dAutoGainMin = 5;   ///< Minimum gain when using AutoGain mode

    m_i64GrayValue = 200;  ///< Expected gray value
  }
} ExposureGain;

/// WhiteBalance
typedef struct WhiteBalance
{
  bool m_bWhiteBalance;                 ///< Auto WhiteBalance is applied ?
  GX_AWB_LAMP_HOUSE_ENTRY lightSource;  ///< The lamp type of environment
  WhiteBalance()
  {
    m_bWhiteBalance = false;                   ///< WhiteBalance is applied defaultly
    lightSource = GX_AWB_LAMP_HOUSE_ADAPTIVE;  ///< Auto adaptive mode
  }
} WhiteBalance;

class GxCamera
{
  // friend void *ProcGetImage(void* pMatImage);

public:
  GxCamera();
  ~GxCamera();

  /// initial
  bool initial(int exposure_time = 1500, float gain = 16.0);

  /// Is open
  bool isOpen();

  /// Close devise
  bool close();

  /// Get exposure time(us)
  void get_exposure_us(double * us);

  // Capture single picture
  bool read(cv::Mat * targetMatImg, float * timestamp);

  /// Open device and display device information
  bool openDevice(bool useSN, const char * CameraSN);

  /// Set trigger mode
  void setTriggerParam(bool useHardTrigger);

  /// Set camera exposure and gain
  void setExposureGainParam(
    bool AutoExposure, bool AutoGain, double ExposureTime, double AutoExposureTimeMin,
    double AutoExposureTimeMax, double Gain, double AutoGainMin, double AutoGainMax,
    int64_t GrayValue);

  /// Set camera roi param
  void setRoiParam(int64_t Width, int64_t Height, int64_t OffsetX, int64_t OffsetY);

  /// Set camera WhiteBalance
  void setWhiteBalanceParam(bool WhiteBalanceOn, GX_AWB_LAMP_HOUSE_ENTRY lightSource);

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
  GX_DEV_HANDLE g_hDevice;     ///< Device handle
  char * g_pDeviceSN;          ///< Device SN number
  int64_t g_i64ColorFilter;    ///< Color filter of device
  int64_t g_nPayloadSize = 0;  ///< Payload size
  uint32_t ui32DeviceNum = 0;  ///< device number

  bool g_bTriggerMode;     ///< Hardware Trigger 1  flag
  bool g_bColorFilter;     ///< Color filter support flag
  bool g_bExposure;        ///< Exposure supprot flag
  bool g_bGain;            ///< Gain supprot flag
  bool g_bImgImprovement;  ///< Image improvement supprot flag
  bool g_bRoi;             ///< Roi support flag
  bool g_bWhiteBalance;    ///< WhiteBalance support flag

  ExposureGain exposure_gain;  ///< Camera exposure and gain param
  Roi roi;                     ///< Camera roi and resolution param
  WhiteBalance white_balance;  ///< Camera whitebalance param
};

extern pthread_mutex_t Globalmutex;  // threads conflict due to image-updating
extern pthread_cond_t GlobalCondCV;  // threads conflict due to image-updating
extern bool imageReadable;           // threads conflict due to image-updating
// extern cv::Mat src;               // Transfering buffer
GxCamera::GxCamera()
{
  g_hDevice = NULL;  ///< Device handle
  g_pDeviceSN = const_cast<char *>("KJ0190120002");
  g_bColorFilter = false;                   ///< Color filter support flag
  g_i64ColorFilter = GX_COLOR_FILTER_NONE;  ///< Color filter of device
  g_nPayloadSize = 0;                       ///< Payload size
}

GxCamera::~GxCamera() {}

bool GxCamera::initial(int exposure_time, float gain)
{
  GX_STATUS status = GX_STATUS_SUCCESS;

  status = GXInitLib();  // 初始化函数库
  GX_VERIFY_EXIT(status);

  status = GXUpdateDeviceList(&ui32DeviceNum, 1000);  // 获取设备数量
  GX_VERIFY_EXIT(status);

  if (ui32DeviceNum <= 0) {
    std::cout << "no devices\n";
    return false;
  }

  // use hardware     Applied 1 normal 0
  setTriggerParam(0);  // 如果要开预测的话，必须要设为1，开启触发模式，C板向相机发送一个触发型号，相机采集一帧图片

  // Attention:   (Width-64)%2=0;   (Height-64)%2=0;  X%16=0;     Y%2=0;
  //    ROI             Width            Height      offset_X    offset_Y
  // setRoiParam(1280, 1024, 0, 0);
  // setRoiParam(1920, 1200, 0, 0);
  // 10   在特别亮的环境下，考虑换成5 正常情况下用10 gain
  //    ExposureGain   autoExposure  autoGain  ExposureTime  AutoExposureMin  AutoExposureMax  Gain(<=16)  AutoGainMin  AutoGainMax  GrayValue
  setExposureGainParam(false, false, exposure_time, 3000, 6000, gain, 5, 16, 127);

  //   WhiteBalance        Applied?       light source type   白平衡  光源类型
  setWhiteBalanceParam(true, GX_AWB_LAMP_HOUSE_ADAPTIVE);

  return true;
}

bool GxCamera::isOpen()  // 判断摄像头是否打开
{
  return ui32DeviceNum >= 0;
}

bool GxCamera::close()  // 关闭设备
{
  GXStreamOff(g_hDevice);
  GXCloseDevice(g_hDevice);
  g_hDevice = NULL;
  GXCloseLib();
  return true;
}

/// Open device and display device information 打开设备并显示图像信息
bool GxCamera::openDevice(bool useSN, const char * CameraSN)
{
  // return GX_STATUS_SUCCESS;
  GX_STATUS status = GX_STATUS_SUCCESS;

  // Get device enumerated number          把这个封装成 isOpened
  if (!isOpen()) {
    printf("<No device found>\n");
    GXCloseLib();
    // exit(status);  // 就是推出
    return false;
  }

  GX_OPEN_PARAM stOpenParam;
  if (useSN) {
    // Init OpenParam , Open device in exclusive mode by SN      //根据SN号打开设备 把是否根据SN号打开改成可选，在函数接口添加参数
    g_pDeviceSN = const_cast<char *>(CameraSN);
    stOpenParam.accessMode =
      GX_ACCESS_EXCLUSIVE;  // 以独占的方式打开设备  GX_ACCESS_CONTROL 以控制的方式打开设备
    stOpenParam.openMode = GX_OPEN_SN;
    stOpenParam.pszContent = g_pDeviceSN;  // SN号，根据此打开设备
  } else {
    // 打开第一个接入的设备
    stOpenParam.accessMode =
      GX_ACCESS_EXCLUSIVE;  // 以独占的方式打开设备  GX_ACCESS_CONTROL 以控制的方式打开设备
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = const_cast<char *>("1");  // SN号，根据此打开设备
  }
  // Open device
  status = GXOpenDevice(&stOpenParam, &g_hDevice);  // 打开设备
  GX_VERIFY_EXIT(status);

  printf("***********************************************\n");

  status = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
  GX_VERIFY_EXIT(status);

  status = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
  GX_VERIFY_EXIT(status);

  // Set trigger
  status = setTrigger();  // 硬件触发模式
  GX_VERIFY_EXIT(status);
  // Set ROI
  status = setRoi();  // 感兴趣区域
  GX_VERIFY_EXIT(status);
  // Set Exposure and Gain
  status = setExposureGain();  // 曝光
  GX_VERIFY_EXIT(status);
  // Set WhiteBalance
  status = setWhiteBalance();  // 白平衡
  GX_VERIFY_EXIT(status);

  status = GXStreamOn(g_hDevice);  // 开始采集
  GX_VERIFY_EXIT(status);

  return true;
}
// struct timeval tb;

bool GxCamera::read(cv::Mat * targetMatImg, float * timestamp)
{
  GX_STATUS emStatus = GX_STATUS_SUCCESS;
  PGX_FRAME_BUFFER pFrameBuffer = NULL;

  emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000);
  if (emStatus != GX_STATUS_SUCCESS && emStatus != GX_STATUS_TIMEOUT) {
    GetErrorString(emStatus);
    return false;
  }

  if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)  // 若为残帧
  {
    printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
    return false;
  } 

  else {
    cv::Mat src;
    src.create(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);  // 为Mat开辟空间
    uchar * pBGRBuf = NULL;
    pBGRBuf = new uchar[pFrameBuffer->nHeight * pFrameBuffer->nWidth * 3];

    // Convert raw8(bayer) image into BGR24 image 将bayer图像转换为RGB图像
    VxInt32 emDXStatus = DX_OK;
    emDXStatus = DxRaw8toRGB24(
      (unsigned char *)pFrameBuffer->pImgBuf, pBGRBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
      RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
    if (emDXStatus != DX_OK) {
      printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
      delete[] pBGRBuf;
      pBGRBuf = NULL;
      return false;
    } else {
      // std::cout << pFrameBuffer->nFrameID << std::endl;
      *timestamp = pFrameBuffer->nTimestamp * 1e-6;
      memcpy(src.data, pBGRBuf, pFrameBuffer->nHeight * pFrameBuffer->nWidth * 3);
      src.copyTo(*targetMatImg);

      // static float time_last = 0;
      // gettimeofday(&tb,NULL);
      // float timeImgae =  tb.tv_usec/1000;
      // std::cout<<"image"<<timeImgae<<std::endl;
      // std::cout<<timeImgae - time_last << std::endl;
      // time_last = timeImgae;

      delete[] pBGRBuf;
      pBGRBuf = NULL;
    }
    emStatus = GXQBuf(g_hDevice, pFrameBuffer);
    if (emStatus != GX_STATUS_SUCCESS) {
      GetErrorString(emStatus);
      return false;
    }
  }
  return true;
}
void GxCamera::get_exposure_us(double * us) { GXGetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, us); }

/// For client
/// Set camera exposure and gain params
void GxCamera::setExposureGainParam(
  bool AutoExposure, bool AutoGain, double ExposureTime, double AutoExposureTimeMin,
  double AutoExposureTimeMax, double Gain, double AutoGainMin, double AutoGainMax,
  int64_t GrayValue)
{
  exposure_gain.m_bAutoExposure = AutoExposure;
  exposure_gain.m_bAutoGain = AutoGain;
  exposure_gain.m_dExposureTime = ExposureTime;
  exposure_gain.m_dAutoExposureTimeMin = AutoExposureTimeMin;
  exposure_gain.m_dAutoExposureTimeMax = AutoExposureTimeMax;
  exposure_gain.m_dGain = Gain;
  exposure_gain.m_dAutoGainMin = AutoGainMin;
  exposure_gain.m_dAutoGainMax = AutoGainMax;
  exposure_gain.m_i64GrayValue = GrayValue;
}

/// Set camera exposure and gain
GX_STATUS GxCamera::setExposureGain()  // 传入mode 参数
{
  GX_STATUS status;
  // if mode reset m_bAutoExposure m_dExposureTime m_dAutoExposureTimeMin m_dAutoExposureTimeMax 重新设置曝光时间 是否自动曝光 自动曝光最小值 自动曝光最大值

  // Set Exposure
  if (exposure_gain.m_bAutoExposure)  // 自动曝光
  {
    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GX_VERIFY(status);
    status =
      GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_gain.m_dAutoExposureTimeMax);
    GX_VERIFY(status);
    status =
      GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_gain.m_dAutoExposureTimeMin);
    GX_VERIFY(status);
  } else  // 非自动曝光
  {
    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    GX_VERIFY(status);
    status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GX_VERIFY(status);
    status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_gain.m_dExposureTime);
    GX_VERIFY(status);
  }

  // Set Gain
  if (exposure_gain.m_bAutoGain) {
    status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GX_VERIFY(status);
    status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_GAIN_MAX, exposure_gain.m_dAutoGainMax);
    GX_VERIFY(status);
    status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_GAIN_MIN, exposure_gain.m_dAutoGainMin);
    GX_VERIFY(status);
  } else {
    status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GX_VERIFY(status);
    status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    GX_VERIFY(status);
    status = GXSetFloat(g_hDevice, GX_FLOAT_GAIN, exposure_gain.m_dGain);
    GX_VERIFY(status);
  }

  // Set Expected Gray Value
  status = GXSetInt(g_hDevice, GX_INT_GRAY_VALUE, exposure_gain.m_i64GrayValue);
  GX_VERIFY(status);

  return status;
}

void GxCamera::setTriggerParam(bool useHardTrigger) { g_bTriggerMode = useHardTrigger; }

// Set trigger mode
GX_STATUS GxCamera::setTrigger()
{
  GX_STATUS emStatus;
  // Set acquisition mode                      //设置采集模式
  emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GX_VERIFY_EXIT(emStatus);

  // Set trigger mode
  if (g_bTriggerMode) {
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    GX_VERIFY_EXIT(emStatus);
  } else {
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(emStatus);
  }

  emStatus = GXSetEnum(g_hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
  GX_VERIFY_EXIT(emStatus);

  emStatus = GXSetEnum(g_hDevice, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);
  GX_VERIFY_EXIT(emStatus);

  // 设置LINE2作为trigger的gpio输入口
  emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
  GX_VERIFY_EXIT(emStatus);

  // Capture single frame
  emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
  GX_VERIFY_EXIT(emStatus);

  return GX_STATUS_SUCCESS;
}

/// For client
/// Set camera roi params               设置相机感兴趣区域参数的变量
void GxCamera::setRoiParam(
  int64_t Width, int64_t Height, int64_t OffsetX,
  int64_t OffsetY)  // 长宽 偏移量   640, 480, 80, 120
{
  roi.m_i64Width = Width;
  roi.m_i64Height = Height;
  roi.m_i64OffsetX = OffsetX;
  roi.m_i64OffsetY = OffsetY;
}

/// Set camera roi
GX_STATUS GxCamera::setRoi()  // setroi
{
  GX_STATUS status = GX_STATUS_SUCCESS;
  // 设 置 一 个 offset 偏 移 为 (X,Y) ,WidthXHeight 尺 寸 的 区 域
  status = GXSetInt(g_hDevice, GX_INT_WIDTH, 64);
  GX_VERIFY(status);  // 确认相机状态
  status = GXSetInt(g_hDevice, GX_INT_HEIGHT, 64);
  GX_VERIFY(status);
  // status = GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, false);
  // std::cout<<status<<std::endl;
  // GX_VERIFY(status);
  status = GXSetInt(g_hDevice, GX_INT_OFFSET_X, roi.m_i64OffsetX);  // 80
  GX_VERIFY(status);
  status = GXSetInt(g_hDevice, GX_INT_OFFSET_Y, roi.m_i64OffsetY);  // 120
  GX_VERIFY(status);
  status = GXSetInt(g_hDevice, GX_INT_WIDTH, roi.m_i64Width);  // 640
  GX_VERIFY(status);
  status = GXSetInt(g_hDevice, GX_INT_HEIGHT, roi.m_i64Height);  // 480
  GX_VERIFY(status);
  return status;
}

/// For client
/// Set camera white balance params
void GxCamera::setWhiteBalanceParam(bool WhiteBalanceOn, GX_AWB_LAMP_HOUSE_ENTRY lightSource)
{
  // 自动白平衡光照环境
  //  GX_AWB_LAMP_HOUSE_ADAPTIVE 自适应
  white_balance.m_bWhiteBalance = WhiteBalanceOn;
  white_balance.lightSource = lightSource;
}

/// Set camera WhiteBalance
GX_STATUS GxCamera::setWhiteBalance()
{
  // 选择白平衡通道
  GX_STATUS status;

  if (white_balance.m_bWhiteBalance) {
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    // status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    // status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RAT IO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);

    // 设置自动白平衡感兴趣区域(整个roi)
    status = GXSetInt(g_hDevice, GX_INT_AWBROI_WIDTH, roi.m_i64Width);
    status = GXSetInt(g_hDevice, GX_INT_AWBROI_HEIGHT, roi.m_i64Height);
    status = GXSetInt(g_hDevice, GX_INT_AWBROI_OFFSETX, 0);
    status = GXSetInt(g_hDevice, GX_INT_AWBROI_OFFSETY, 0);
    GX_VERIFY(status);

    // 自动白平衡设置
    status = GXSetEnum(g_hDevice, GX_ENUM_AWB_LAMP_HOUSE, white_balance.lightSource);
    GX_VERIFY(status);

    // 设置连续自动白平衡
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    GX_VERIFY(status);
  } else {
    status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GX_VERIFY(status);
  }

  return GX_STATUS_SUCCESS;
}

GX_STATUS GxCamera::setBufferSize()
{
  GX_STATUS emStatus;

  // Set buffer quantity of acquisition queue  设置采集队列的缓冲数量
  uint64_t nBufferNum = ACQ_BUFFER_NUM;
  emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
  GX_VERIFY_EXIT(emStatus);

  bool bStreamTransferSize = false;
  emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
  GX_VERIFY_EXIT(emStatus);

  if (bStreamTransferSize) {
    // Set size of data transfer block      USB3 Vision相机传输时每个数据块的大小   (64 * 1024)
    emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
    GX_VERIFY_EXIT(emStatus);
  }

  bool bStreamTransferNumberUrb = false;  // GXIsImplemented函数的作用是查询当前函数是否支持此功能
  emStatus =
    GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
  GX_VERIFY_EXIT(emStatus);

  if (bStreamTransferNumberUrb) {
    // Set qty. of data transfer block       设置数据传输块的大小 限制了单个设备实际映射到系统内核的数据块数量  64
    emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
    GX_VERIFY_EXIT(emStatus);
  }
  return GX_STATUS_SUCCESS;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
  char * error_info = NULL;
  size_t size = 0;
  GX_STATUS emStatus = GX_STATUS_SUCCESS;

  // Get length of error description
  emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
  if (emStatus != GX_STATUS_SUCCESS) {
    printf("<Error when calling GXGetLastError>\n");
    return;
  }

  // Alloc error resources
  error_info = new char[size];
  if (error_info == NULL) {
    printf("<Failed to allocate memory>\n");
    return;
  }

  // Get error description
  emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
  if (emStatus != GX_STATUS_SUCCESS) {
    printf("<Error when calling GXGetLastError>\n");
  } else {
    printf("%s\n", (char *)error_info);
  }

  // Realease error resources
  if (error_info != NULL) {
    delete[] error_info;
    error_info = NULL;
  }
}

#endif  // !RM_VERSION4_AUTOAIM_INCLUDE_GXCAMERA_HPP_
