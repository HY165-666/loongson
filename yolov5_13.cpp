#include "layer.h"
#include "net.h"

#if defined(USE_NCNN_SIMPLEOCV)
#include "simpleocv.h"
#else
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif
#include <float.h>
#include <stdio.h>
#include <vector>
#include <algorithm>

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>

// ========== 调试宏 ==========
#define DEBUG_PRINT_ENABLE  1   // 1 开启打印，0 关闭（比赛时可关闭）
#if DEBUG_PRINT_ENABLE
    #define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

// ========== 摄像头配置 ==========
#define CAMERA_DEVICE   "/dev/video0"
#define IMG_WIDTH       640
#define IMG_HEIGHT      480
#define BUFFER_COUNT    4

// ========== 串口配置 ==========
#define SERIAL_DEVICE   "/dev/ttyS3"
#define SERIAL_BAUD     B115200
#define SERIAL_DATABITS 8
#define SERIAL_PARITY   'n'
#define SERIAL_STOPBITS 1

static int serial_fd = -1;
static volatile int keep_running = 1;

// ========== 检测框结构 ==========
struct Object {
    cv::Rect_<float> rect;
    int label;
    float prob;
};

// ========== 函数声明 ==========
static void yuyv_to_bgr(const uint8_t* yuyv, cv::Mat& bgr, int width, int height);
static int capture_frame(cv::Mat& frame);
static int serial_init(const char* device, int baud, int databits, char parity, int stopbits);
static int serial_read_byte(int fd);
static void serial_write_byte(int fd, unsigned char ch);
static void signal_handler(int sig);
static float adc_to_lux(int adc);
static void send_json_via_tcp(const std::string& json_str);
static void parse_and_send(const std::string& frame);

// ========== 模型解析函数 ==========
static void parse_out0(const ncnn::Mat& out, float prob_threshold, std::vector<Object>& objects)
{
    const int num_dets = out.h;
    const int num_features = out.w;
    const int num_class = num_features - 5;

    fprintf(stderr, "parse_out0: dets=%d, features=%d, classes=%d\n", num_dets, num_features, num_class);
    for (int k = 0; k < 3 && k < num_dets; k++)
    {
        const float* det = out.row(k);
        fprintf(stderr, "  det[%d]: cx=%.4f cy=%.4f w=%.4f h=%.4f obj=%.4f",
                k, det[0], det[1], det[2], det[3], det[4]);
        for (int c = 0; c < num_class; c++)
            fprintf(stderr, " cls%d=%.4f", c, det[5 + c]);
        fprintf(stderr, "\n");
    }

    const float SOFT_MAX = 20.0f;
    for (int k = 0; k < num_dets; k++)
    {
        const float* det = out.row(k);

        float obj_raw = det[4];
        if (obj_raw > SOFT_MAX) obj_raw = SOFT_MAX;
        if (obj_raw < -SOFT_MAX) obj_raw = -SOFT_MAX;
        float obj_conf = 1.0f / (1.0f + expf(-obj_raw));

        int class_index = 0;
        float class_score_raw = -FLT_MAX;
        for (int c = 0; c < num_class; c++)
        {
            float score_raw = det[5 + c];
            if (score_raw > SOFT_MAX) score_raw = SOFT_MAX;
            if (score_raw < -SOFT_MAX) score_raw = -SOFT_MAX;
            float score = 1.0f / (1.0f + expf(-score_raw));
            if (score > class_score_raw)
            {
                class_score_raw = score;
                class_index = c;
            }
        }

        float confidence = obj_conf * class_score_raw;
        float final_threshold = (prob_threshold < 0.05f) ? 0.05f : prob_threshold;
        if (confidence >= final_threshold)
        {
            float cx = det[0];
            float cy = det[1];
            float bw = det[2];
            float bh = det[3];
            if (cx <= 0 || cy <= 0 || bw <= 0 || bh <= 0) continue;

            float x0 = cx - bw * 0.5f;
            float y0 = cy - bh * 0.5f;
            float x1 = cx + bw * 0.5f;
            float y1 = cy + bh * 0.5f;

            Object obj;
            obj.rect.x = x0;
            obj.rect.y = y0;
            obj.rect.width = x1 - x0;
            obj.rect.height = y1 - y0;
            obj.label = class_index;
            obj.prob = confidence;
            objects.push_back(obj);
        }
    }
    fprintf(stderr, "parse_out0: generated %zu proposals (threshold=%.3f)\n", objects.size(), prob_threshold);
}

// ========== NMS 函数 ==========
static float intersection_area(const Object& a, const Object& b) {
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = faceobjects.size();
    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) areas[i] = faceobjects[i].rect.area();
    for (int i = 0; i < n; i++) {
        const Object& a = faceobjects[i];
        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++) {
            const Object& b = faceobjects[picked[j]];
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            if (inter_area / union_area > nms_threshold) {
                keep = 0;
                break;
            }
        }
        if (keep) picked.push_back(i);
    }
}

// ========== 推理函数 ==========
int inference_and_print(cv::Mat& bgr, ncnn::Net& net, int target_size = 320,
                         float prob_threshold = 0.45f, float nms_threshold = 0.45f) {
    int img_w = bgr.cols, img_h = bgr.rows;
    int w = img_w, h = img_h;
    float scale = 1.f;
    if (w > h) {
        scale = (float)target_size / w;
        w = target_size;
        h = h * scale;
    } else {
        scale = (float)target_size / h;
        h = target_size;
        w = w * scale;
    }
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB,
                                                  img_w, img_h, w, h);
    int wpad = target_size - w;
    int hpad = target_size - h;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2,
                           wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in_pad);
    ncnn::Mat out;
    int ret = ex.extract("out0", out);
    if (ret != 0 || out.empty()) {
        fprintf(stderr, "extract out0 failed, ret=%d, empty=%d\n", ret, out.empty());
        return 0;
    }
    fprintf(stderr, "extract success, out shape: w=%d h=%d c=%d\n", out.w, out.h, out.c);

    std::vector<Object> proposals;
    parse_out0(out, prob_threshold, proposals);

    std::sort(proposals.begin(), proposals.end(),
              [](const Object& a, const Object& b) { return a.prob > b.prob; });
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    for (int idx : picked) {
        Object obj = proposals[idx];
        float x0 = (obj.rect.x - wpad / 2) / scale;
        float y0 = (obj.rect.y - hpad / 2) / scale;
        float x1 = (obj.rect.x + obj.rect.width - wpad / 2) / scale;
        float y1 = (obj.rect.y + obj.rect.height - hpad / 2) / scale;
        x0 = std::max(x0, 0.f);
        y0 = std::max(y0, 0.f);
        x1 = std::min(x1, (float)img_w);
        y1 = std::min(y1, (float)img_h);
        if (x1 <= x0 || y1 <= y0) continue;

        cv::rectangle(bgr, cv::Point((int)x0, (int)y0), cv::Point((int)x1, (int)y1), cv::Scalar(0, 255, 0), 2);
        char text[64];
        sprintf(text, "%d:%.2f", obj.label, obj.prob);
        cv::putText(bgr, text, cv::Point((int)x0, (int)y0 - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        printf("Label: %d, Prob: %.4f, Box: [%.1f, %.1f, %.1f, %.1f]\n",
               obj.label, obj.prob, x0, y0, x1 - x0, y1 - y0);
    }
    printf("Total detections after NMS: %d\n", (int)picked.size());
    return (int)picked.size();
}

// ========== YUYV 转 BGR ==========
static void yuyv_to_bgr(const uint8_t* yuyv, cv::Mat& bgr, int width, int height) {
    bgr.create(height, width, CV_8UC3);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 2) {
            int idx = i * width + j;
            uint8_t y0 = yuyv[idx * 2];
            uint8_t u  = yuyv[idx * 2 + 1];
            uint8_t y1 = yuyv[idx * 2 + 2];
            uint8_t v  = yuyv[idx * 2 + 3];

            int c0 = y0 - 16;
            int c1 = y1 - 16;
            int d = u - 128;
            int e = v - 128;

            int b0 = (298 * c0 + 516 * d + 128) >> 8;
            int g0 = (298 * c0 - 100 * d - 208 * e + 128) >> 8;
            int r0 = (298 * c0 + 409 * e + 128) >> 8;

            int b1 = (298 * c1 + 516 * d + 128) >> 8;
            int g1 = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
            int r1 = (298 * c1 + 409 * e + 128) >> 8;

            uint8_t* bgr_data = bgr.data;
            bgr_data[idx * 3]     = (b0 < 0) ? 0 : (b0 > 255) ? 255 : b0;
            bgr_data[idx * 3 + 1] = (g0 < 0) ? 0 : (g0 > 255) ? 255 : g0;
            bgr_data[idx * 3 + 2] = (r0 < 0) ? 0 : (r0 > 255) ? 255 : r0;
            bgr_data[(idx + 1) * 3]     = (b1 < 0) ? 0 : (b1 > 255) ? 255 : b1;
            bgr_data[(idx + 1) * 3 + 1] = (g1 < 0) ? 0 : (g1 > 255) ? 255 : g1;
            bgr_data[(idx + 1) * 3 + 2] = (r1 < 0) ? 0 : (r1 > 255) ? 255 : r1;
        }
    }
}

// ========== 摄像头抓帧 ==========
static int capture_frame(cv::Mat& frame) {
    int fd = open("/dev/video0", O_RDWR);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    int ret = -1;
    void* buffers[4] = {NULL};
    size_t buf_lengths[4] = {0};
    struct v4l2_buffer buf;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    enum v4l2_buf_type type;
    fd_set fds;
    struct timeval tv;
    int sel_ret;
    bool streamon = false;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        goto cleanup;
    }
    printf("Format set: %dx%d, YUYV\n", fmt.fmt.pix.width, fmt.fmt.pix.height);

    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        goto cleanup;
    }
    if (req.count < 1) {
        fprintf(stderr, "Insufficient buffer memory\n");
        goto cleanup;
    }

    for (int i = 0; i < req.count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            goto cleanup;
        }
        buf_lengths[i] = buf.length;
        buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, buf.m.offset);
        if (buffers[i] == MAP_FAILED) {
            perror("mmap");
            buffers[i] = NULL;
            goto cleanup;
        }
    }

    for (int i = 0; i < req.count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            goto cleanup;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        goto cleanup;
    }
    streamon = true;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    sel_ret = select(fd + 1, &fds, NULL, NULL, &tv);
    if (sel_ret < 0) {
        perror("select");
        goto cleanup;
    } else if (sel_ret == 0) {
        fprintf(stderr, "select timeout\n");
        goto cleanup;
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("VIDIOC_DQBUF");
        goto cleanup;
    }

    yuyv_to_bgr((uint8_t*)buffers[buf.index], frame, 640, 480);
    ret = 0;

    ioctl(fd, VIDIOC_QBUF, &buf);
    if (streamon) {
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        streamon = false;
    }

cleanup:
    if (streamon) {
        enum v4l2_buf_type type2 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMOFF, &type2);
    }
    for (int i = 0; i < 4; i++) {
        if (buffers[i] != NULL && buf_lengths[i] > 0) {
            munmap(buffers[i], buf_lengths[i]);
        }
    }
    close(fd);
    return ret;
}

// ========== 串口相关函数 ==========
static int serial_init(const char* device, int baud, int databits, char parity, int stopbits) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open serial");
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&options);
    speed_t baud_rate;
    switch (baud) {
        case B115200: baud_rate = B115200; break;
        case B9600:   baud_rate = B9600;   break;
        default:      baud_rate = B115200; break;
    }
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag &= ~CSIZE;
    if (databits == 8) options.c_cflag |= CS8;
    else if (databits == 7) options.c_cflag |= CS7;
    else return -1;

    if (parity == 'n' || parity == 'N') {
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
    } else if (parity == 'o' || parity == 'O') {
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
    } else if (parity == 'e' || parity == 'E') {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
    } else return -1;

    if (stopbits == 1) options.c_cflag &= ~CSTOPB;
    else if (stopbits == 2) options.c_cflag |= CSTOPB;
    else return -1;

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    return fd;
}

static int serial_read_byte(int fd) {
    unsigned char ch;
    int n = read(fd, &ch, 1);
    if (n == 1) return ch;
    else if (n == 0) return -1;
    else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return -1;
        return -2;
    }
}

static void serial_write_byte(int fd, unsigned char ch) {
    write(fd, &ch, 1);
}

// ========== 信号处理 ==========
static void signal_handler(int sig) {
    (void)sig;
    keep_running = 0;
}

// ========== ADC 转 Lux ==========
/**
 * ADC 值转 Lux（分段拟合，适用于常见光敏电阻 + 10kΩ 分压电路）
 * @param adc 0~4095
 * @return 光照度 (Lux)
 */
static float adc_to_lux(int adc) {
    // --- 1. 边界保护 ---
    if (adc <= 0) return 0.0f;
    if (adc >= 4095) return 10000.0f;  // 强光上限

    // --- 2. 计算光敏电阻阻值 (单位 kΩ) ---
    const float R_FIXED = 10.0f;  // 分压电阻 10kΩ
    float R_photo = R_FIXED * (4095.0f - adc) / adc;  // 单位 kΩ

    // --- 3. 分段拟合 ---
    float lux;
    if (R_photo >= 10.0f) {
        // 低照度：阻值大，使用幂函数拟合
        lux = 500.0f / powf(R_photo, 0.8f);
    } else if (R_photo >= 1.0f) {
        // 中照度：阻值适中，使用线性插值过渡
        lux = 45.0f / R_photo + 5.0f;
    } else {
        // 高照度：阻值小，使用反比修正
        lux = 50.0f / (R_photo + 0.1f);
    }

    // --- 4. 限制输出范围 ---
    if (lux < 0.0f) lux = 0.0f;
    if (lux > 10000.0f) lux = 10000.0f;
    return lux;
}

// ========== TCP 发送 JSON（带调试打印） ==========
static void send_json_via_tcp(const std::string& json_str) {
    DEBUG_PRINT("send_json_via_tcp() called, json length: %zu\n", json_str.size());
    DEBUG_PRINT("JSON content: %s\n", json_str.c_str());

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("[TCP] socket creation failed");
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
        DEBUG_PRINT("[TCP] Connected to 127.0.0.1:12345\n");
        std::string msg = json_str + "\n";
        int sent = send(sock, msg.c_str(), msg.size(), 0);
        if (sent > 0) {
            DEBUG_PRINT("[TCP] Sent %d bytes\n", sent);
        } else {
            perror("[TCP] send failed");
        }
    } else {
        perror("[TCP] connect failed");
    }

    close(sock);
    DEBUG_PRINT("[TCP] socket closed\n");
}

// ========== 解析传感器数据帧并发送（带调试打印） ==========
static void parse_and_send(const std::string& frame) {
    DEBUG_PRINT("parse_and_send() called, frame = '%s'\n", frame.c_str());

    if (frame.size() < 5) {
        DEBUG_PRINT("Frame too short, ignored.\n");
        return;
    }
    std::string content = frame.substr(1, frame.size() - 2); // 去掉 # 和 $
    std::vector<std::string> fields;
    size_t pos = 0;
    while ((pos = content.find(',')) != std::string::npos) {
        fields.push_back(content.substr(0, pos));
        content.erase(0, pos + 1);
    }
    fields.push_back(content);
    if (fields.size() != 9) {
        DEBUG_PRINT("Field count %zu, expected 9. Ignored.\n", fields.size());
        return;
    }

    try {
        int d1 = std::stoi(fields[0]);
        int s1 = std::stoi(fields[1]);
        int d2 = std::stoi(fields[2]);
        int s2 = std::stoi(fields[3]);
        int d3 = std::stoi(fields[4]);
        int s3 = std::stoi(fields[5]);
        int adc = std::stoi(fields[6]);
        float temp = std::stof(fields[7]);
        float hum = std::stof(fields[8]);

        DEBUG_PRINT("Parsed: d1=%d s1=%d d2=%d s2=%d d3=%d s3=%d adc=%d temp=%.1f hum=%.1f\n",
                    d1, s1, d2, s2, d3, s3, adc, temp, hum);

        float p1 = 1.5f * d1 / 100.0f;
        float p2 = 1.5f * d2 / 100.0f;
        float p3 = 1.5f * d3 / 100.0f;
        float lux = adc_to_lux(adc);

        char json_buf[512];
        snprintf(json_buf, sizeof(json_buf),
            "{\"lamp1\":{\"duty\":%d,\"status\":%d,\"power\":%.2f},"
            "\"lamp2\":{\"duty\":%d,\"status\":%d,\"power\":%.2f},"
            "\"lamp3\":{\"duty\":%d,\"status\":%d,\"power\":%.2f},"
            "\"env\":{\"light\":%.2f,\"temp\":%.1f,\"hum\":%.1f}}",
            d1, s1, p1,
            d2, s2, p2,
            d3, s3, p3,
            lux, temp, hum
        );
        DEBUG_PRINT("Constructed JSON: %s\n", json_buf);
        send_json_via_tcp(std::string(json_buf));
    } catch (const std::exception& e) {
        DEBUG_PRINT("Parse exception: %s\n", e.what());
    } catch (...) {
        DEBUG_PRINT("Unknown parse exception\n");
    }
}

// ========== 主函数 ==========
int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    if (argc == 1) {
        printf("No arguments provided. Entering serial monitor mode.\n");
        printf("Waiting for 'A' on %s ...\n", SERIAL_DEVICE);

        serial_fd = serial_init(SERIAL_DEVICE, SERIAL_BAUD, SERIAL_DATABITS, SERIAL_PARITY, SERIAL_STOPBITS);
        if (serial_fd < 0) {
            fprintf(stderr, "Failed to open serial port %s\n", SERIAL_DEVICE);
            return -1;
        }
        printf("Serial ready.\n");

        ncnn::Net net;
        net.opt.use_vulkan_compute = false;
        if (net.load_param("1-opt.param") != 0 || net.load_model("1-opt.bin") != 0) {
            fprintf(stderr, "Model load failed\n");
            close(serial_fd);
            return -1;
        }
        printf("Model loaded.\n");

        fd_set readfds;
        struct timeval tv;
        int max_fd = serial_fd + 1;

        cv::Mat frame;
        std::string frame_buffer;  // 用于累积传感器数据帧

        while (keep_running) {
            FD_ZERO(&readfds);
            FD_SET(serial_fd, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 100000;

            int ret = select(max_fd, &readfds, NULL, NULL, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                perror("select");
                break;
            } else if (ret == 0) {
                continue;
            }

            if (FD_ISSET(serial_fd, &readfds)) {
                int ch = serial_read_byte(serial_fd);
                if (ch < 0) continue;

                // 打印接收到的字节（十六进制和字符）
                DEBUG_PRINT("Received byte: 0x%02X ('%c')\n", ch, (ch >= 32 && ch <= 126) ? (char)ch : '?');

                if (ch == 'A') {
                    // 原有功能：拍照推理
                    printf("Received 'A'! Capturing and inferencing...\n");
                    if (capture_frame(frame) == 0) {
                        int det_count = inference_and_print(frame, net);
                        cv::imwrite("camera_result.jpg", frame);
                        printf("Result saved, detections: %d\n", det_count);
                        if (det_count > 0) {
                            serial_write_byte(serial_fd, 'B');
                            printf("Sent 'B'\n");
                        }
                    }
                } else if (ch == '#') {
                    DEBUG_PRINT("Frame start marker '#' received\n");
                    frame_buffer.clear();
                    frame_buffer += (char)ch;
                } else if (ch == '$') {
                    DEBUG_PRINT("Frame end marker '$' received, buffer size: %zu\n", frame_buffer.size());
                    frame_buffer += (char)ch;
                    if (!frame_buffer.empty()) {
                        parse_and_send(frame_buffer);
                    } else {
                        DEBUG_PRINT("Empty frame, ignored.\n");
                    }
                    frame_buffer.clear();
                } else {
                    // 非控制字符，如果正在接收数据帧则累积
                    if (!frame_buffer.empty()) {
                        frame_buffer += (char)ch;
                    }
                    // 防止缓冲区过长
                    if (frame_buffer.size() > 1024) {
                        DEBUG_PRINT("Buffer overflow, cleared.\n");
                        frame_buffer.clear();
                    }
                }
            }
        }

        close(serial_fd);
        printf("Serial closed.\n");
        return 0;
    }

    fprintf(stderr, "Usage: %s\n", argv[0]);
    fprintf(stderr, "       (no arguments) -> serial monitor mode\n");
    return -1;
}
