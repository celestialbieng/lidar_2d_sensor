#ifndef UST10LX_H
#define UST10LX_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <array>
#include <string>
#include <cmath> 
#include "DataPoint.h"

constexpr float degreeToRadian = M_PI/180.0f;

class UST10LX
{
public:
    static constexpr uint16_t scanPoints = 1081;
    static constexpr int16_t invalidDistance = -1;
    static constexpr float angularResolution = 0.25f;

    explicit UST10LX(float angleOffset = 0.0f);
    ~UST10LX();

    bool connect(const std::string& ip, uint16_t port = 10940);
    bool scan();
    operator bool();

    const std::vector<DataPoint>& getDataPoints() const;
    std::vector<DataPoint> getDataPointsFast();
    const std::array<int16_t, scanPoints>& getScan() const;
    uint32_t getDataPointCount() const;

private:
    int16_t decodeDistanceData(uint16_t start, uint8_t charLength);
    bool sendCommand(const std::string& message);
    uint16_t readResponse(uint16_t bytesToRead = 0);

    static constexpr uint16_t defaultLidarPort = 10940;
    static constexpr uint16_t minValidDistance = 10;
    static constexpr uint16_t maxValidDistance = 3000;

    const float m_angleOffsetRad;
    int m_lidarSocket = -1;
    sockaddr_in m_lidarAddress;
    std::string m_receiveBuffer;
    std::array<int16_t, scanPoints> m_distanceData;
    std::vector<DataPoint> m_dataPointScan;
    uint32_t m_dataPointCount = 0;

    void clearScanData(){
        m_distanceData.fill(invalidDistance);
        m_dataPointCount = 0;
    }

    std::array<int16_t, 3> m_distanceFilterBuffer;
    int16_t applyDistanceFilter(int16_t raw) {
        m_distanceFilterBuffer[0] = m_distanceFilterBuffer[1];
        m_distanceFilterBuffer[1] = m_distanceFilterBuffer[2];
        m_distanceFilterBuffer[2] = raw;
        
        if(raw == 0) return 0;
        
        if(m_distanceFilterBuffer[0] > 0 && 
           m_distanceFilterBuffer[1] > 0 && 
           m_distanceFilterBuffer[2] > 0) {
            return (m_distanceFilterBuffer[0] + 
                   m_distanceFilterBuffer[1] + 
                   m_distanceFilterBuffer[2]) / 3;
        }
        return raw;
    }
};

#endif // UST10LX_H