#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <iostream>
#include <filesystem>
#include <algorithm>

#include <windows.h>
#include <sstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace cv;
using namespace cv::dnn;
using namespace std;

static double lastPrintT = 0.0;

// Servo state (degrees)
static float panDeg  = 90.0f;   // start middle
static float tiltDeg = 0.0f;   // start horizontal

static const float PAN_MIN  = 0.0f;
static const float PAN_MAX  = 180.0f;
static const float TILT_MIN = 0.0f;
static const float TILT_MAX = 180.0f;

static const float PAN_SIGN  = -1.0f;
static const float TILT_SIGN = -1.0f;

// Tracking parameters 
static const int   DEAD_X = 10;     // px deadband
static const int   DEAD_Y = 10;     // px deadband
static const float SMOOTH_A = 0.20f;  // 0..1 (higher = smoother, more lag)

// Filtered center
static bool  haveFilt = false;
static float uf = 0.0f, vf = 0.0f;

// Send-rate limiting (servo update rate)
static const double SERVO_HZ = 50.0; // 50 updates/sec
static double lastSendT = 0.0;

struct SerialOut {
    HANDLE h = INVALID_HANDLE_VALUE;

    explicit SerialOut(const std::string& comName, int baud = 115200) {
        // For COM10+ use "\\\\.\\COM10". For COM5 "COM5" also works, but we normalize.
        std::string full = comName;
        if (full.rfind("COM", 0) == 0 && full.size() > 4) {
            // COM10, COM11... must be prefixed
            full = "\\\\.\\" + full;
        }

        h = CreateFileA(full.c_str(),
                        GENERIC_WRITE,
                        0,               // no sharing
                        NULL,
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL,
                        NULL);

        if (h == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Failed to open serial port " + comName);
        }

        // Configure serial port
        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(h, &dcb)) {
            CloseHandle(h);
            h = INVALID_HANDLE_VALUE;
            throw std::runtime_error("GetCommState failed");
        }

        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary  = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;

        if (!SetCommState(h, &dcb)) {
            CloseHandle(h);
            h = INVALID_HANDLE_VALUE;
            throw std::runtime_error("SetCommState failed");
        }

        // Low-latency timeouts (we only write)
        COMMTIMEOUTS timeouts{};
        timeouts.WriteTotalTimeoutConstant = 10;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(h, &timeouts);

        // clear buffers
        PurgeComm(h, PURGE_TXCLEAR | PURGE_RXCLEAR);
    }

    ~SerialOut() {
        if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
    }

    void sendAngles(float pan, float tilt) {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(1);
        ss << pan << "," << tilt << "\n";
        std::string msg = ss.str();

        DWORD written = 0;
        BOOL ok = WriteFile(h, msg.c_str(), (DWORD)msg.size(), &written, NULL);
        if (!ok || written != msg.size()) {
            // throw std::runtime_error("WriteFile failed");
        }
    }
};

static inline float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(v, hi));
}

// Step size in degrees based on absolute pixel error
static inline float stepDegFromErr(int aerrPx)
{
    if (aerrPx > 200) return 3.0f;
    if (aerrPx > 80)  return 1.5f;
    if (aerrPx > 20)  return 0.5f;
    return 0.0f; // inside deadband region (or near)
}

static inline Rect clipRect(const Rect& r, int W, int H)
{
    int x = max(0, min(r.x, W - 1));
    int y = max(0, min(r.y, H - 1));
    int w = max(0, min(r.width,  W - x));
    int h = max(0, min(r.height, H - y));
    return Rect(x, y, w, h);
}

// Letterbox to 640x640 without stretching (keeps aspect ratio).
// Returns the padded image + scale + padding offsets used.
static void letterbox(const Mat& srcBGR, Mat& dstBGR, int netW, int netH,
                      float& scale, int& padX, int& padY)
{
    scale = min(netW / (float)srcBGR.cols, netH / (float)srcBGR.rows);
    int newW = (int)round(srcBGR.cols * scale);
    int newH = (int)round(srcBGR.rows * scale);

    Mat resized;
    resize(srcBGR, resized, Size(newW, newH));

    padX = (netW - newW) / 2;
    padY = (netH - newH) / 2;

    dstBGR = Mat(netH, netW, CV_8UC3, Scalar(114, 114, 114));
    resized.copyTo(dstBGR(Rect(padX, padY, newW, newH)));
}

// Convert whatever OpenCV gives us into a 2D matrix: [num_preds x num_cols]
static Mat to2D(const Mat& out)
{
    // Common cases we handle:
    // 1) [1, N, C]  -> N x C
    // 2) [1, C, N]  -> transpose -> N x C
    // 3) [N, C]     -> already
    // 4) [1,1,N,C]  -> squeeze -> N x C
    // 5) [1,1,C,N]  -> squeeze then transpose -> N x C
    if (out.dims == 2)
        return out;

    if (out.dims == 3 && out.size[0] == 1)
    {
        int d1 = out.size[1];
        int d2 = out.size[2];
        Mat tmp(d1, d2, CV_32F, (void*)out.ptr<float>(0));
        // if "channels" is small (< 100) and "preds" is large, it's probably [1, C, N]
        if (d1 <= 100 && d2 > 100)
            return tmp.t();   // N x C
        else
            return tmp;       // N x C
    }

    if (out.dims == 4 && out.size[0] == 1 && out.size[1] == 1)
    {
        int d2 = out.size[2];
        int d3 = out.size[3];
        Mat tmp(d2, d3, CV_32F, (void*)out.ptr<float>(0));
        if (d2 <= 100 && d3 > 100)
            return tmp.t();
        else
            return tmp;
    }

    Mat flat = out.reshape(1, 1);
    return flat;
}

// Parse YOLO-like output and return the best box (after NMS) in ORIGINAL frame coords.
// Works for the most common YOLOv8 ONNX layouts where first 4 are box values.
static bool detectOneBallSquare(
    const Mat& frameBGR,
    Net& net,
    Rect& outSquare,
    float confThresh = 0.35f,
    float nmsThresh  = 0.45f,
    int netW = 640,
    int netH = 640
)
{
    // Preprocess with letterbox
    Mat lb;
    float scale = 1.0f;
    int padX = 0, padY = 0;
    letterbox(frameBGR, lb, netW, netH, scale, padX, padY);

    Mat blob = blobFromImage(lb, 1.0 / 255.0, Size(netW, netH), Scalar(), true, false);
    net.setInput(blob);

    vector<Mat> outs;
    net.forward(outs, net.getUnconnectedOutLayersNames());
    if (outs.empty())
        return false;

    Mat det2d = to2D(outs[0]);  // [num_preds x num_cols]
    if (det2d.empty())
        return false;

    // Collect candidates
    vector<Rect> boxes;
    vector<float> scores;

    const int rows = det2d.rows;
    const int cols = det2d.cols;

    for (int i = 0; i < rows; ++i)
    {
        const float* p = det2d.ptr<float>(i);

        float a = p[0], b = p[1], c = p[2], d = p[3];

        // detect normalized coords
        bool normalized = (fabs(a) <= 2.0f && fabs(b) <= 2.0f && fabs(c) <= 2.0f && fabs(d) <= 2.0f);

        // Two common encodings:
        //  - xywh (center x,y,w,h)
        //  - xyxy (x1,y1,x2,y2)
        bool looks_xyxy = (c > a) && (d > b) && ( (normalized) || (c <= netW + 5 && d <= netH + 5) );

        float x1, y1, x2, y2;

        if (looks_xyxy)
        {
            x1 = a; y1 = b; x2 = c; y2 = d;
            if (normalized) { x1 *= netW; y1 *= netH; x2 *= netW; y2 *= netH; }
        }
        else
        {
            float cx = a, cy = b, w = c, h = d;
            if (normalized) { cx *= netW; cy *= netH; w *= netW; h *= netH; }
            x1 = cx - w * 0.5f;
            y1 = cy - h * 0.5f;
            x2 = cx + w * 0.5f;
            y2 = cy + h * 0.5f;
        }

        // Confidence handling:
        // - Many exports: [x,y,w,h,conf,class] or [x,y,w,h,obj,cls...]
        float score = 0.0f;

        if (cols >= 6)
        {
            float v4 = p[4];
            float v5 = p[5];

            // If v5 looks like a probability, combine; otherwise treat v4 as final confidence.
            if (v5 >= 0.0f && v5 <= 1.0f)
                score = v4 * v5;
            else
                score = v4;
        }
        else if (cols == 5)
        {
            score = p[4];
        }
        else
        {
            continue;
        }

        if (score < confThresh)
            continue;

        // Undo letterbox: move from net coords back to original frame coords
        x1 = (x1 - padX) / scale;
        y1 = (y1 - padY) / scale;
        x2 = (x2 - padX) / scale;
        y2 = (y2 - padY) / scale;

        int left   = (int)round(x1);
        int top    = (int)round(y1);
        int width  = (int)round(x2 - x1);
        int height = (int)round(y2 - y1);

        if (width <= 2 || height <= 2)
            continue;

        Rect r(left, top, width, height);
        r = clipRect(r, frameBGR.cols, frameBGR.rows);
        if (r.width <= 2 || r.height <= 2)
            continue;

        boxes.push_back(r);
        scores.push_back(score);
    }

    if (boxes.empty())
        return false;

    // NMS to remove duplicates
    vector<int> keep;
    NMSBoxes(boxes, scores, confThresh, nmsThresh, keep);
    if (keep.empty())
        return false;

    // Pick best remaining (highest score)
    int bestIdx = keep[0];
    for (int k : keep)
        if (scores[k] > scores[bestIdx]) bestIdx = k;

    Rect b = boxes[bestIdx];

    // Convert to a SQUARE box around the detection
    Point2f center(b.x + b.width * 0.5f, b.y + b.height * 0.5f);
    int side = max(b.width, b.height);
    Rect sq((int)round(center.x - side * 0.5f),
            (int)round(center.y - side * 0.5f),
            side, side);

    sq = clipRect(sq, frameBGR.cols, frameBGR.rows);
    if (sq.width <= 2 || sq.height <= 2)
        return false;

    outSquare = sq;
    return true;
}

int main()
{
    SerialOut esp("COM5", 115200);

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    namedWindow("Frame", WINDOW_NORMAL);

    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();

    if (camList.GetSize() == 0)
    {
        cout << "No cameras found.\n";
        system->ReleaseInstance();
        return -1;
    }

    CameraPtr cam = camList.GetByIndex(0);
    cam->Init();
    INodeMap& nodeMap = cam->GetNodeMap();

    // disable auto white balance
    try {
        CEnumerationPtr wbAuto(nodeMap.GetNode("BalanceWhiteAuto"));
        if (IsAvailable(wbAuto) && IsWritable(wbAuto)) {
            CEnumEntryPtr off = wbAuto->GetEntryByName("Off");
            if (IsAvailable(off) && IsReadable(off)) {
                wbAuto->SetIntValue(off->GetValue());
                std::cout << "BalanceWhiteAuto = Off\n";
            }
        }
    } catch (...) {
        std::cout << "Warning: couldn't set BalanceWhiteAuto\n";
    }

    // Low-lag buffer mode
    try {
        cam->TLStream.StreamBufferHandlingMode.SetValue(StreamBufferHandlingMode_NewestOnly);
    } catch (...) {
        cout << "Warning: can't set StreamBufferHandlingMode_NewestOnly\n";
    }

    // Exposure
    try {
        cam->ExposureAuto.SetValue(ExposureAuto_Off);
        cam->ExposureTime.SetValue(20300);
    } catch (...) {
        cout << "Warning: can't set exposure settings\n";
    }

    // Bayer for speed
    try {
        cam->PixelFormat.SetValue(PixelFormat_BayerRG8);
    } catch (...) {
        cout << "Warning: can't set PixelFormat_BayerRG8.\n";
    }

    cam->BeginAcquisition();

    ImageProcessor processor;
    processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

    // Load ONNX
    const string onnxPath = "C:/Users/alexb/Desktop/TRACKER/models/best.onnx";
    cout << "Working dir: " << filesystem::current_path().string() << "\n";
    if (!filesystem::exists(onnxPath)) {
        cerr << "ONNX not found: " << onnxPath << "\n";
        return -1;
    }

    Net net;
    try {
        net = readNetFromONNX(onnxPath);
    } catch (const cv::Exception& e) {
        cerr << "Failed to load ONNX:\n" << e.what() << "\n";
        return -1;
    }

    if (net.empty()) {
        cerr << "Net is empty after loading ONNX.\n";
        return -1;
    }

    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    cout << "Streaming + YOLO\n";

    lastSendT = (double)getTickCount() / getTickFrequency();

    while (true)
    {
        ImagePtr raw = cam->GetNextImage(100);
        if (raw->IsIncomplete())
        {
            raw->Release();
            continue;
        }

        ImagePtr converted = processor.Convert(raw, PixelFormat_BGR8);

        Mat frame(converted->GetHeight(), converted->GetWidth(), CV_8UC3, converted->GetData());
        if (frame.empty())
        {
            converted->Release();
            raw->Release();
            continue;
        }

        // IMPORTANT: clone so we can safely use it after releasing Spinnaker buffers
        Mat frameCopy = frame.clone();

        // release ASAP (low-lag & avoid buffer buildup)
        converted->Release();
        raw->Release();

        // detect one ball square
        Rect sq;
        bool ok = detectOneBallSquare(frameCopy, net, sq, 0.35f, 0.45f, 640, 640);

        int cx = frameCopy.cols / 2;
        int cy = frameCopy.rows / 2;

        // draw center of frame
        circle(frameCopy, Point(cx, cy), 4, Scalar(255, 255, 0), 2);

        if (ok)
        {
            rectangle(frameCopy, sq, Scalar(0, 255, 0), 2);

            Point center(sq.x + sq.width / 2, sq.y + sq.height / 2);
            circle(frameCopy, center, 3, Scalar(0, 0, 255), FILLED);

            // 1) Smooth the detected center
            if (!haveFilt) {
                uf = (float)center.x;
                vf = (float)center.y;
                haveFilt = true;
            } else {
                uf = SMOOTH_A * uf + (1.0f - SMOOTH_A) * (float)center.x;
                vf = SMOOTH_A * vf + (1.0f - SMOOTH_A) * (float)center.y;
            }

            Point filtCenter((int)std::round(uf), (int)std::round(vf));
            circle(frameCopy, filtCenter, 5, Scalar(0, 255, 255), 2); // filtered center (yellow)

            // 2) Compute pixel errors relative to frame center
            int ex = filtCenter.x - cx;
            int ey = filtCenter.y - cy;

            // show errors for debugging
            putText(frameCopy, format("ex=%d ey=%d", ex, ey), Point(10, 30),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);

            // 3) Deadband (don’t move if close enough)
            int aex = std::abs(ex);
            int aey = std::abs(ey);

            float stepPan  = (aex <= DEAD_X) ? 0.0f : stepDegFromErr(aex);
            float stepTilt = (aey <= DEAD_Y) ? 0.0f : stepDegFromErr(aey);

            // 4) Update pan/tilt degrees incrementally (direction from sign of error)
            if (stepPan > 0.0f) {
                float dir = (ex > 0) ? +1.0f : -1.0f;          // ball right => move right (usually)
                panDeg += PAN_SIGN * dir * stepPan;
            }
            if (stepTilt > 0.0f) {
                float dir = (ey > 0) ? +1.0f : -1.0f;          // ball down => move down (usually)
                tiltDeg += TILT_SIGN * dir * stepTilt;
            }

            panDeg  = clampf(panDeg,  PAN_MIN,  PAN_MAX);
            tiltDeg = clampf(tiltDeg, TILT_MIN, TILT_MAX);
        }
        else
        {
        }

        double nowT = (double)getTickCount() / getTickFrequency();
        double dtSend = nowT - lastSendT;

        if (dtSend >= (1.0 / SERVO_HZ))
        {
            lastSendT = nowT;
            double nowT2 = (double)getTickCount() / getTickFrequency();
            if (nowT2 - lastPrintT > 1.0) {
                lastPrintT = nowT2;
                cout << "SEND " << panDeg << "," << tiltDeg << "\n";
            }
            try {
                esp.sendAngles(panDeg, tiltDeg);
            } catch (...) {
                // throw std::runtime_error("Failed to send angles");
            }

            putText(frameCopy, format("PAN=%.1f TILT=%.1f", panDeg, tiltDeg),
                    Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
        }

        imshow("Frame", frameCopy);

        int key = waitKey(1);
        if (key == 27) break; // ESC
    }

    cam->EndAcquisition();
    cam->DeInit();
    camList.Clear();
    system->ReleaseInstance();
    return 0;
}
