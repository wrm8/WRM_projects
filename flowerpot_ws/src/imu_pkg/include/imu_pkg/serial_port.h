#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <sys/ioctl.h>

class SerialPort {
public:
    SerialPort() : fd_(-1) {}
    
    ~SerialPort() { if (isOpen()) close(); }
    
    void setPort(const std::string& port) { port_ = port; }
    
    void setBaudrate(int baudrate) { baudrate_ = baudrate; }
    
    void setTimeout(int /*timeout_ms*/) {}
    
    bool open() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) return false;
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd_, &tty);
        
        speed_t speed;
        switch (baudrate_) {
            case 9600: speed = B9600; break;
            case 115200: speed = B115200; break;
            default: speed = B9600;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        
        tcsetattr(fd_, TCSANOW, &tty);
        return true;
    }
    
    bool isOpen() const { return fd_ != -1; }
    
    void close() { if (fd_ != -1) { ::close(fd_); fd_ = -1; } }
    
    size_t available() {
        if (fd_ == -1) return 0;
        int bytes = 0;
        ioctl(fd_, FIONREAD, &bytes);
        return bytes > 0 ? bytes : 0;
    }
    
    size_t read(uint8_t* buffer, size_t size) {
        if (fd_ == -1) return 0;
        return ::read(fd_, buffer, size);
    }
    
private:
    int fd_;
    std::string port_;
    int baudrate_ = 9600;
};

#endif