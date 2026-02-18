#include "../include/UST10LX.h"
#include <iostream>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <cerrno>
#include <vector>

UST10LX::UST10LX(float angleOffset) : 
    m_angleOffsetRad(angleOffset * degreeToRadian)
{
    m_dataPointScan.reserve(scanPoints);
    for(int i=0; i<scanPoints; i++) {
        DataPoint point;
        point.angle = 0.0;
        point.distance = 0;
        m_dataPointScan.push_back(point);
    }
}

UST10LX::~UST10LX() {
    if(m_lidarSocket != -1) {
        std::string stopCommand = "QT\n";
        while(::write(m_lidarSocket, stopCommand.c_str(), stopCommand.size()) != stopCommand.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ::close(m_lidarSocket);
    }
}

std::vector<DataPoint> UST10LX::getDataPointsFast() {
    std::vector<DataPoint> result;
    result.reserve(scanPoints);
    
    for(uint16_t i = 0; i < scanPoints; i++) {
        if(m_distanceData[i] > minValidDistance && m_distanceData[i] < maxValidDistance) {
            float angle = (i * angularResolution - 135.0f) * degreeToRadian + m_angleOffsetRad;
            DataPoint point;
            point.angle = angle;
            point.distance = m_distanceData[i];
            result.push_back(point);
        }
    }
    return result;
}

bool UST10LX::connect(const std::string& ip, uint16_t port) {
    // Close existing connection if any
    if(m_lidarSocket != -1) {
        ::close(m_lidarSocket);
        m_lidarSocket = -1;
    }

    // Create new socket
    m_lidarSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(m_lidarSocket == -1) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return false;
    }

    // Configure address
    m_lidarAddress.sin_family = AF_INET;
    m_lidarAddress.sin_port = htons(port);
    m_lidarAddress.sin_addr.s_addr = inet_addr(ip.c_str());

    // Set timeout options (5 seconds)
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(m_lidarSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(m_lidarSocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // Attempt connection
    if(::connect(m_lidarSocket, reinterpret_cast<sockaddr*>(&m_lidarAddress), sizeof(m_lidarAddress))) {
        std::cerr << "Connection to " << ip << ":" << port << " failed: " << strerror(errno) << std::endl;
        ::close(m_lidarSocket);
        m_lidarSocket = -1;
        return false;
    }

    // Initialize LiDAR
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(!sendCommand("BM")) {
        std::cerr << "Initialization command failed" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readResponse();

    return true;
}

bool UST10LX::scan() {
    clearScanData();

    if(!sendCommand("GD0000108000")) return false;
    if(!readResponse()) return false;

    if(m_receiveBuffer.size() < 23 + 3*scanPoints) {
        return false;
    }

    size_t endPosition = m_receiveBuffer.find("\n\n");
    if(endPosition == std::string::npos) return false;
    m_receiveBuffer.erase(endPosition-1, 3);

    m_receiveBuffer.erase(0, 23);
    if(m_receiveBuffer.empty()) return false;

    endPosition = m_receiveBuffer.find('\n');
    while(endPosition != std::string::npos) {
        m_receiveBuffer.erase(endPosition-1, 2);
        endPosition = m_receiveBuffer.find('\n');
    }

    auto distanceIt = m_distanceData.begin();
    auto pointIt = m_dataPointScan.begin();
    m_dataPointCount = 0;

    for(uint16_t i = 0; i < m_receiveBuffer.size(); i += 3) {
        int16_t rawDistance = decodeDistanceData(i, 3);

        if(rawDistance <= minValidDistance || rawDistance >= maxValidDistance) {
            rawDistance = 0;
            continue;
        }

        float angleRad = (i/3.0f * angularResolution - 135.0f) * degreeToRadian + m_angleOffsetRad;
        int16_t filteredDistance = applyDistanceFilter(rawDistance);

        *distanceIt = filteredDistance;
        ++distanceIt;

        pointIt->distance = filteredDistance;
        pointIt->angle = angleRad;
        ++pointIt;

        m_dataPointCount++;
    }

    m_dataPointScan.resize(m_dataPointCount);
    return true;
}

int16_t UST10LX::decodeDistanceData(uint16_t start, uint8_t charLength)
{
    int16_t value = 0;
    auto position = m_receiveBuffer.begin() + start;
    for(int i = 0; i < charLength; i++)
    {
        value += (*position-0x30) << 6*(charLength-i-1);
        ++position;
    }
    return value;
}

const std::array<int16_t, UST10LX::scanPoints>& UST10LX::getScan() const
{
    return m_distanceData;
}

const std::vector<DataPoint>& UST10LX::getDataPoints() const
{
    return m_dataPointScan;
}

uint16_t UST10LX::readResponse(uint16_t bytesToRead)
{
    if(m_lidarSocket == -1) return 0;

    uint16_t bytesRead = 0;
    char currentChar = '\0', previousChar ='\0';
    m_receiveBuffer.clear();

    if(bytesToRead)
    {
        while(bytesRead < bytesToRead)
        {
            bytesRead += ::read(m_lidarSocket, &currentChar, 1);
            m_receiveBuffer.push_back(currentChar);
        }
    }
    else
    {
        // Read until message end: "\n\n"
        while(currentChar != '\n' || previousChar != '\n')
        {
            previousChar = currentChar;
            bytesRead += ::read(m_lidarSocket, &currentChar, 1);
            m_receiveBuffer.push_back(currentChar);
        }
    }

    return bytesRead;
}

bool UST10LX::sendCommand(const std::string& message)
{
    if(m_lidarSocket == -1) return false;

    std::string command = message;
    command.append("\n");

    uint16_t sentBytes = ::write(m_lidarSocket, command.c_str(), command.size());
    return (sentBytes == command.size());
}

uint32_t UST10LX::getDataPointCount() const
{
    return m_dataPointCount;
}

UST10LX::operator bool() {
    return m_lidarSocket != -1;
}