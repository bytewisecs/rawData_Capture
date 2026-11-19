#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <limits>
#include <string>
#ifdef _WIN32
    #include <Winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include "WzSerialportPlus/Windows/WzSerialportPlus.h"
#elif __linux__
    #include <sys/socket.h>
    #include <sys/select.h>
    #include <netinet/in.h>
    #include "WzSerialportPlus/Linux/WzSerialportPlus.h"
#endif

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

WzSerialportPlus radar_serial_port;
int memidx=0;
int overflowCnt=0;
int packetNum_g=1000, packetSize_g=1456;
uint8_t *serial_buf_ptr=NULL;
std::mutex serial_buf_mutex;

int add(int i, int j) {
    return i+j;
}

bool radar_start_read_thread(std::string portName, int baudrate, int packetNum, int packetSize){
    if(serial_buf_ptr!=NULL)
        return false;
    serial_buf_ptr = new uint8_t[packetSize*packetNum];//initialize buffer
    packetNum_g=packetNum;
    packetSize_g=packetSize;

    radar_serial_port.setReceiveCalback([&](char* data,int length){//Set the receive callback function and execute it after receiving the data
        serial_buf_mutex.lock();//exclusive buffer
        if(memidx+length>packetSize_g*packetNum_g){//Guaranteed not to exceed the buffer range
            overflowCnt++;
            printf("[fpga_udp]serial buffer overflowed for %d time(s)!memidx=%d,length=%d,packetSize=%d,packetNum=%d\n",overflowCnt,memidx,length,packetSize_g,packetNum_g);
            memidx=0;
        }
        memcpy(serial_buf_ptr+memidx,data,length);//Copy the received data into the buffer
        memidx+=length;
        serial_buf_mutex.unlock();
    });

    #ifdef _WIN32
        if (portName.substr(0,3)=="COM" && stoi(portName.substr(3))>=10)
            portName="\\\\.\\"+portName;//port name has prefix in windows
    #endif
    return radar_serial_port.open(portName,baudrate,1,8,'n');//Open the port to start the thread to receive data
}

py::array_t<uint8_t> get_radar_buf(){
    serial_buf_mutex.lock();//exclusive buffer
    auto result = py::array_t<uint8_t>(memidx);
    py::buffer_info buf = result.request();
    uint8_t *buf_ptr = static_cast<uint8_t *>(buf.ptr);
    memcpy(buf_ptr,serial_buf_ptr,memidx);
    memidx=0;
    serial_buf_mutex.unlock();
    return result;
}

void radar_stop_read_thread(){
    radar_serial_port.close();
    serial_buf_mutex.lock();
    delete[] serial_buf_ptr;
    serial_buf_ptr=NULL;
    memidx=0;
    overflowCnt=0;
    serial_buf_mutex.unlock();
}

py::array_t<uint8_t> read_data_udp(int sock_fd, int packetNum, int packetSize, double timeout_s){
    if (packetNum < 0 || packetSize <= 0 || packetSize > 65507 || packetSize % 2 != 0)
        throw std::invalid_argument("[fpga_udp]Invalid packet count/size");
    if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > std::numeric_limits<int>::max())
        throw std::invalid_argument("[fpga_udp]timeout_s must be positive and finite");
    if (sock_fd < 0)
        throw std::invalid_argument("[fpga_udp]Invalid socket descriptor");
#ifdef _WIN32
    SOCKET sockfd = static_cast<SOCKET>(sock_fd);
#else
    int sockfd = sock_fd;
    if (sockfd >= FD_SETSIZE)
        throw std::invalid_argument("[fpga_udp]Socket descriptor exceeds select limit");
#endif
    if (packetNum > (std::numeric_limits<py::ssize_t>::max() - 1) / packetSize)
        throw std::overflow_error("[fpga_udp]Receive buffer is too large");
    const py::ssize_t capacity = static_cast<py::ssize_t>(packetNum) * packetSize;
    // One extra byte detects oversized datagrams instead of silently truncating.
    auto storage = py::array_t<uint8_t>(capacity + 1);
    auto *buffer = storage.mutable_data();
    py::ssize_t received_bytes = 0;
    {
        // Allocation and Python objects stay outside this scope. Capture timers,
        // DSP and record workers can run while this native receiver waits.
        py::gil_scoped_release release;
        const auto started = std::chrono::steady_clock::now();
        while (received_bytes < capacity) {
            const double remaining = timeout_s - std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            if (remaining <= 0) break;
            timeval wait;
            wait.tv_sec = static_cast<long>(remaining);
            wait.tv_usec = static_cast<long>((remaining - wait.tv_sec) * 1000000);
            fd_set readable;
            FD_ZERO(&readable);
            FD_SET(sockfd, &readable);
#ifdef _WIN32
            const int ready = select(0, &readable, nullptr, nullptr, &wait);
#else
            const int ready = select(sockfd + 1, &readable, nullptr, nullptr, &wait);
#endif
            if (ready == 0) break;
            if (ready < 0) {
#ifdef _WIN32
                const int error = WSAGetLastError();
                if (error == WSAEINTR) continue;
#else
                const int error = errno;
                if (error == EINTR) continue;
#endif
                throw std::runtime_error("[fpga_udp]select failed: " + std::to_string(error));
            }
            const int length = recvfrom(sockfd, reinterpret_cast<char *>(buffer + received_bytes),
                                        packetSize + 1, 0, nullptr, nullptr);
            if (length < 0) {
#ifdef _WIN32
                const int error = WSAGetLastError();
                if (error == WSAEINTR || error == WSAEWOULDBLOCK) continue;
#else
                const int error = errno;
                if (error == EINTR || error == EAGAIN || error == EWOULDBLOCK) continue;
#endif
                throw std::runtime_error("[fpga_udp]recvfrom failed: " + std::to_string(error));
            }
            if (length != packetSize)
                throw std::runtime_error("[fpga_udp]Unexpected UDP packet length: " + std::to_string(length));
            received_bytes += length;
        }
    } // GIL reacquired before creating the returned NumPy view.
    // Keep storage alive through the base reference, exposing ONLY received bytes.
    return py::array_t<uint8_t>(received_bytes, buffer, storage);
}

PYBIND11_MODULE(fpga_udp, m) {
    m.doc() = R"pbdoc(
        FPGA UDP reader plugin
        -----------------------

        .. currentmodule:: fpga_udp

        .. autosummary::
           :toctree: _generate

           add
           subtract
           radar_start_read_thread
           get_radar_buf
           radar_stop_read_thread
           getOverflowCnt
           read_data_udp
    )pbdoc";

    m.attr("read_data_udp_api_version") = 2; // valid-length result, batch deadline, released GIL

    m.def("radar_start_read_thread", &radar_start_read_thread);
    m.def("get_radar_buf", &get_radar_buf);
    m.def("radar_stop_read_thread", &radar_stop_read_thread);
    m.def("getOverflowCnt", []() {
        return overflowCnt;
    });
    m.def("reset_radar_buf", []() {
        serial_buf_mutex.lock();
        memidx=0;
        overflowCnt=0;
        serial_buf_mutex.unlock();
    });

    m.def("read_data_udp", &read_data_udp, R"pbdoc(
        read UDP packet from FPGA as fast as written in C.

        sockfd:\tan open file descriptor of a socket
        packetNum:\tA total of packetNum packets will be read
        packetSize:\tand the size of each packet is packetSize
        timeout_s:\ttotal batch timeout in seconds (fractional values accepted).
        Returns only complete received packets as a flat uint8 array; empty on timeout.
    )pbdoc");

    m.def("add", &add, R"pbdoc(
        Add two numbers
    )pbdoc");

    m.def("subtract", [](int i, int j) {
        return i - j;
    });

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
