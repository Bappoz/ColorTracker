#include "v4l2_source.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace ecv::linux_hal {
namespace {

/// ioctl retorna EINTR quando um sinal chega no meio — reenviar, não falhar.
int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

uint32_t to_v4l2_format(PixelFormat f) {
    switch (f) {
        case PixelFormat::kYuyv: return V4L2_PIX_FMT_YUYV;
        case PixelFormat::kRgb565: return V4L2_PIX_FMT_RGB565;
        case PixelFormat::kBgr888: return V4L2_PIX_FMT_BGR24;
        case PixelFormat::kRgb888: return V4L2_PIX_FMT_RGB24;
    }
    return V4L2_PIX_FMT_YUYV;
}

void set_control(int fd, uint32_t id, int32_t value) {
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;
    xioctl(fd, VIDIOC_S_CTRL, &ctrl);  // controle ausente no driver não é fatal
}

bool ecv_can_decode(uint32_t fourcc) {
    return fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_RGB565 ||
           fourcc == V4L2_PIX_FMT_BGR24 || fourcc == V4L2_PIX_FMT_RGB24;
}

}  // namespace

int list_modes(const std::string& device, V4l2Mode* out, int max_modes) {
    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd == -1) return -1;

    int count = 0;
    v4l2_fmtdesc desc{};
    desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (; count < max_modes && xioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0; ++desc.index) {
        v4l2_frmsizeenum size{};
        size.pixel_format = desc.pixelformat;
        for (; count < max_modes && xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
            if (size.type != V4L2_FRMSIZE_TYPE_DISCRETE) continue;

            V4l2Mode& m = out[count];
            for (int i = 0; i < 4; ++i) {
                m.fourcc[i] = static_cast<char>((desc.pixelformat >> (8 * i)) & 0xFF);
            }
            m.width = static_cast<int32_t>(size.discrete.width);
            m.height = static_cast<int32_t>(size.discrete.height);
            m.supported_by_ecv = ecv_can_decode(desc.pixelformat);

            v4l2_frmivalenum interval{};
            interval.pixel_format = desc.pixelformat;
            interval.width = size.discrete.width;
            interval.height = size.discrete.height;
            if (xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval) == 0 &&
                interval.type == V4L2_FRMIVAL_TYPE_DISCRETE && interval.discrete.numerator) {
                m.fps = interval.discrete.denominator / interval.discrete.numerator;
            }
            ++count;
        }
    }

    ::close(fd);
    return count;
}

bool V4l2Source::fail(const char* what) {
    error_ = std::string(what) + ": " + std::strerror(errno);
    close();
    return false;
}

bool V4l2Source::set_format() {
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<uint32_t>(cfg_.width);
    fmt.fmt.pix.height = static_cast<uint32_t>(cfg_.height);
    fmt.fmt.pix.pixelformat = to_v4l2_format(cfg_.format);
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) return fail("VIDIOC_S_FMT");

    // O driver pode negociar outra coisa: aceitar o que ele devolveu, não o que
    // foi pedido, senão o stride fica errado e a imagem sai enviesada.
    cfg_.width = static_cast<int32_t>(fmt.fmt.pix.width);
    cfg_.height = static_cast<int32_t>(fmt.fmt.pix.height);
    stride_ = static_cast<int32_t>(fmt.fmt.pix.bytesperline);
    if (fmt.fmt.pix.pixelformat != to_v4l2_format(cfg_.format)) {
        error_ = "driver não aceitou o formato de pixel pedido";
        close();
        return false;
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = cfg_.fps;
    xioctl(fd_, VIDIOC_S_PARM, &parm);
    return true;
}

bool V4l2Source::request_buffers() {
    v4l2_requestbuffers req{};
    req.count = cfg_.buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) return fail("VIDIOC_REQBUFS");
    if (req.count < 2) {
        error_ = "driver não forneceu buffers suficientes";
        close();
        return false;
    }

    buffer_count_ = req.count > 8 ? 8 : req.count;
    for (uint32_t i = 0; i < buffer_count_; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) return fail("VIDIOC_QUERYBUF");

        buffers_[i].length = buf.length;
        buffers_[i].start =
            ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
            buffers_[i].start = nullptr;
            return fail("mmap");
        }
    }
    return true;
}

bool V4l2Source::start_streaming() {
    for (uint32_t i = 0; i < buffer_count_; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) return fail("VIDIOC_QBUF");
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) return fail("VIDIOC_STREAMON");
    return true;
}

void V4l2Source::apply_camera_controls() {
    if (cfg_.lock_exposure) {
        set_control(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        set_control(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, cfg_.exposure_absolute);
        set_control(fd_, V4L2_CID_AUTO_WHITE_BALANCE, 0);
        set_control(fd_, V4L2_CID_AUTOGAIN, 0);
    } else {
        set_control(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY);
        set_control(fd_, V4L2_CID_AUTO_WHITE_BALANCE, 1);
        set_control(fd_, V4L2_CID_AUTOGAIN, 1);
    }
}

void V4l2Source::set_exposure_lock(bool locked) {
    cfg_.lock_exposure = locked;
    if (fd_ != -1) apply_camera_controls();
}

bool V4l2Source::open() {
    fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ == -1) return fail(cfg_.device.c_str());

    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) return fail("VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING)) {
        error_ = "dispositivo não faz captura por streaming";
        close();
        return false;
    }

    if (!set_format()) return false;
    apply_camera_controls();
    if (!request_buffers()) return false;
    return start_streaming();
}

void V4l2Source::close() {
    if (fd_ != -1) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (uint32_t i = 0; i < buffer_count_; ++i) {
        if (buffers_[i].start) ::munmap(buffers_[i].start, buffers_[i].length);
        buffers_[i] = Buffer{};
    }
    buffer_count_ = 0;
    queued_index_ = -1;
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool V4l2Source::next(ImageView& out) {
    if (fd_ == -1) return false;

    // Devolve o buffer do frame anterior só agora: enquanto o pipeline estava
    // processando, o driver continuou preenchendo os outros.
    if (queued_index_ >= 0) {
        v4l2_buffer rebuf{};
        rebuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rebuf.memory = V4L2_MEMORY_MMAP;
        rebuf.index = static_cast<uint32_t>(queued_index_);
        if (xioctl(fd_, VIDIOC_QBUF, &rebuf) == -1) return fail("VIDIOC_QBUF");
        queued_index_ = -1;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv{};
    tv.tv_sec = 2;
    if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return fail("select");

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) return fail("VIDIOC_DQBUF");

    queued_index_ = static_cast<int>(buf.index);
    out = ImageView{static_cast<const uint8_t*>(buffers_[buf.index].start), cfg_.width, cfg_.height,
                    stride_, cfg_.format};
    return true;
}

}  // namespace ecv::linux_hal
