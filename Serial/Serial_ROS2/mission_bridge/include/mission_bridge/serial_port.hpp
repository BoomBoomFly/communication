/*
 * @file    serial_port.hpp
 * @brief   POSIX 串口封装头文件
 * @author  Wanone111
 * @note    该文件定义了最小 termios 串口封装，8N1 无流控，支持常用波特率，
 *          读操作带 VTIME/VMIN 超时，不依赖任何外部库。
 */

#ifndef MISSION_BRIDGE_SERIAL_PORT_HPP
#define MISSION_BRIDGE_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace mission_bridge
{

/*
 * @brief  最小 POSIX termios 串口封装类。
 * @note   该类不可拷贝，读超时固定为 0.5s（VTIME=5, VMIN=0），便于读线程周期检查退出标志。
 */
class SerialPort
{
public:
    /*
     * @brief  构造函数，初始为未打开状态。
     */
    SerialPort();

    /*
     * @brief  构造函数，直接打开并配置串口。
     * @param  path: 串口设备路径，如 /dev/ttyUSB0。
     * @param  baudrate: 波特率，支持 9600~921600 常用档位。
     * @note   打开方式与 serial_driver_ros 的 SerialComm(port, baudrate) 接口风格一致；
     *         打开失败不抛异常，用 isOpen() 查询结果。
     */
    SerialPort(const std::string &path, int baudrate);

    /*
     * @brief  析构函数，自动关闭串口。
     */
    ~SerialPort();

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    /*
     * @brief  打开并配置串口。
     * @param  path: 串口设备路径，如 /dev/ttyUSB0。
     * @param  baudrate: 波特率，支持 9600~921600 常用档位。
     * @retval true 表示打开并配置成功，false 表示失败。
     * @note   配置为 8N1、无流控，VMIN=0、VTIME=5（0.5s 读超时）。
     */
    bool open(const std::string &path, int baudrate);

    /*
     * @brief  关闭串口。
     * @retval None
     * @note   重复调用安全。
     */
    void close();

    /*
     * @brief  查询串口是否已打开。
     * @param  None
     * @retval true 表示串口已打开。
     */
    bool isOpen() const;

    /*
     * @brief  从串口读取数据。
     * @param  buf: 接收缓冲区。
     * @param  len: 缓冲区长度。
     * @retval 实际读取字节数；0 表示读超时；-1 表示读错误，错误码见 errno。
     */
    ssize_t read(uint8_t *buf, size_t len);

    /*
     * @brief  向串口写入数据，尽量写完全部字节。
     * @param  buf: 发送缓冲区。
     * @param  len: 发送长度。
     * @retval 实际写入字节数；-1 表示写错误，错误码见 errno。
     */
    ssize_t write(const uint8_t *buf, size_t len);

private:
    int fd_; /* 串口文件描述符，-1 表示未打开 */
};

} // namespace mission_bridge

#endif /* MISSION_BRIDGE_SERIAL_PORT_HPP */
