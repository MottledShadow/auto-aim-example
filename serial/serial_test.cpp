#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main()
{
    const char* device = "/dev/ttyTHS1";

    // 1. 打开串口
    int fd = open(device, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        std::cerr << "open failed: " << std::strerror(errno) << '\n';
        return 1;
    }

    // 2. 读取当前串口配置
    termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: "
                  << std::strerror(errno) << '\n';
        close(fd);
        return 1;
    }

    // 3. 设置波特率
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // 4. 设置为 raw 模式
    cfmakeraw(&tty);

    // 5. 8N1
    tty.c_cflag &= ~PARENB;   // 无校验
    tty.c_cflag &= ~CSTOPB;   // 1 个停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 数据位

    // 6. 不使用硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 允许接收
    tty.c_cflag |= CREAD | CLOCAL;

    // read() 行为：
    // 至少收到 1 个字节才返回
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    // 7. 把配置写回串口
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: "
                  << std::strerror(errno) << '\n';
        close(fd);
        return 1;
    }

    std::cout << "UART opened: " << device << '\n';

    // 8. 先发送一句测试数据
    const char msg[] = "hello from jetson\r\n";

    if (write(fd, msg, sizeof(msg) - 1) < 0) {
        std::cerr << "write failed\n";
    }

    // 9. 持续接收
    unsigned char buffer[256];

    while (true) {
        ssize_t n = read(fd, buffer, sizeof(buffer));

        if (n < 0) {
            std::cerr << "read failed: "
                      << std::strerror(errno) << '\n';
            break;
        }

        std::cout << "received " << n << " bytes: ";

        for (ssize_t i = 0; i < n; ++i) {
            printf("%02X ", buffer[i]);
        }

        std::cout << '\n';

        // 收到多少就原样回多少
        if (write(fd, buffer, n) < 0) {
            std::cerr << "write failed: "
                      << std::strerror(errno) << '\n';
            break;
        }
    }

    close(fd);
    return 0;
}