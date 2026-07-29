/*
 * @file    serial_port.cpp
 * @brief   POSIX 串口封装实现文件
 * @author  Wanone111
 * @note    该文件实现了基于 termios 的最小串口封装，8N1 无流控，
 *          读超时为 VTIME=5、VMIN=0（0.5s），不依赖任何外部库。
 */

#include "mission_bridge/serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace mission_bridge
{

namespace
{

/*
 * @brief  将整数波特率转换为 termios 速率常量。
 * @param  baudrate: 整数波特率。
 * @param  speed: 输出 termios 速率常量。
 * @retval true 表示支持该波特率。
 */
bool BaudToSpeed(int baudrate, speed_t &speed)
{
    switch (baudrate)
    {
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;
        case 230400: speed = B230400; return true;
        case 460800: speed = B460800; return true;
        case 921600: speed = B921600; return true;
        default:                      return false;
    }
}

} // namespace

SerialPort::SerialPort() : fd_(-1)
{
}

SerialPort::SerialPort(const std::string &path, int baudrate) : fd_(-1)
{
    (void)open(path, baudrate);
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string &path, int baudrate)
{
    speed_t speed;

    close();
    if (!BaudToSpeed(baudrate, speed))
    {
        return false;
    }

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        return false;
    }

    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0)
    {
        close();
        return false;
    }

    /* 原始模式：8N1、无流控 */
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    (void)cfsetispeed(&tio, speed);
    (void)cfsetospeed(&tio, speed);

    /* VMIN=0、VTIME=5：read 最多阻塞 0.5s，超时返回 0 */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 5;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
        close();
        return false;
    }

    (void)tcflush(fd_, TCIOFLUSH);

    /* 清除 O_NONBLOCK，使 read 按 VTIME/VMIN 阻塞等待 */
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
    {
        (void)fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0)
    {
        (void)::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const
{
    return fd_ >= 0;
}

ssize_t SerialPort::read(uint8_t *buf, size_t len)
{
    if (fd_ < 0 || buf == nullptr || len == 0U)
    {
        errno = EINVAL;
        return -1;
    }

    return ::read(fd_, buf, len);
}

ssize_t SerialPort::write(const uint8_t *buf, size_t len)
{
    size_t written = 0U;

    if (fd_ < 0 || (buf == nullptr && len > 0U))
    {
        errno = EINVAL;
        return -1;
    }

    while (written < len)
    {
        ssize_t ret = ::write(fd_, buf + written, len - written);
        if (ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return (written > 0U) ? (ssize_t)written : -1;
        }
        written += (size_t)ret;
    }

    return (ssize_t)written;
}

} // namespace mission_bridge
