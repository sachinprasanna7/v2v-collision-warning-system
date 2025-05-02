#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string.h>

#define YELLOW_CODE "\033[33m"
#define TEAL_CODE "\033[36m"
#define BOLD_CODE "\033[1m"
#define END_CODE "\033[0m"
#define RED_CODE "\033[31m"

using namespace ns3; 
using namespace std;

NS_LOG_COMPONENT_DEFINE("Wave_Project");

struct NeighborEntry
{
    Mac48Address mac;
    double distance;
};

map<Mac48Address, vector<NeighborEntry>> neighborTable;
double WARNING_DISTANCE = 5.0; 

// Function to print the locations of all vehicles
void
PrintVehicleLocations(NodeContainer& vehicles)
{
    cout << "\n** Vehicle Locations **\n";
    cout << "Time: " << Simulator::Now().GetSeconds() << " seconds\n";
    cout << "---------------------------------\n";
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        Ptr<Node> node = vehicles.Get(i);
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();

        Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(node->GetDevice(0));
        Mac48Address mac = Mac48Address::ConvertFrom(device->GetAddress());

        cout << "Vehicle " << mac << " , " << i + 1 << " Location: (" << pos.x << ", " << pos.y
             << ", " << pos.z << ") meters\n";
    }
}


// Function to print the neighbor table of all vehicles
void
PrintNeighborTable(NodeContainer& vehicles)
{
    cout << "\n** Neighbor Table Information **\n";
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        Ptr<Node> node = vehicles.Get(i);
        Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(node->GetDevice(0));
        Mac48Address srcMac = Mac48Address::ConvertFrom(device->GetAddress());

        cout << "Vehicle " << srcMac << " , " << i + 1 << " Neighbor Table:\n";
        for (const auto& entry : neighborTable[srcMac])
        {
            cout << "  Neighbor MAC: " << entry.mac << ", Distance: " << entry.distance
                 << " meters\n";
        }
    }
}

// Function to send a collision warning packet to a vehicle

// device is the sender here, targetMac is the receiver of the packet
void
SendCollisionWarning(Ptr<WaveNetDevice> device, Mac48Address targetMac)
{
    Mac48Address senderMac = Mac48Address::ConvertFrom(device->GetAddress());
    Mac48Address receiverMac = targetMac;

    std::ostringstream senderStream, receiverStream;
    senderStream << senderMac;
    receiverStream << receiverMac;

    std::string senderStr = senderStream.str();
    std::string receiverStr = receiverStream.str();

    std::string warningMsg = "Collision Warning: Vehicle " + senderStr +
                             " is dangerously close to Vehicle " + receiverStr +
                             ". Reduce the speed!";

    uint32_t msgLength = warningMsg.size();
    std::unique_ptr<uint8_t[]> msgBuffer(new uint8_t[msgLength]);
    std::memcpy(msgBuffer.get(), warningMsg.c_str(), msgLength);

    Ptr<Packet> packet = Create<Packet>(msgBuffer.get(), msgLength);

    TxInfo txInfo;
    txInfo.preamble = WIFI_PREAMBLE_LONG;
    txInfo.channelNumber = CCH;  // Control Channel (178)
    txInfo.priority = 7;         // High priority , 7 is the highest priority
    txInfo.txPowerLevel = 3;
    txInfo.dataRate = WifiMode("OfdmRate12MbpsBW10MHz");

    Mac48Address broadcast = Mac48Address::GetBroadcast();
    uint16_t wsmpProtocol = 0x88dc;

    device->SendX(packet, broadcast, wsmpProtocol, txInfo);
}


void
SendHello(Ptr<WaveNetDevice> netDevice, int vehicleIndex)
{
    // Retrieve current position of the node
    Ptr<MobilityModel> mobility = netDevice->GetNode()->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();

    // Construct the message with vehicle index and coordinates
    std::ostringstream messageStream;
    messageStream << "Hello from vehicle " << vehicleIndex 
                  << " x:" << pos.x 
                  << " y:" << pos.y;
    std::string message = messageStream.str();

    // Convert the message to byte buffer
    uint32_t msgSize = message.size();
    std::unique_ptr<uint8_t[]> msgBuffer(new uint8_t[msgSize]);
    std::memcpy(msgBuffer.get(), message.c_str(), msgSize);

    // Prepare transmission info
    TxInfo txInfo;
    txInfo.preamble = WIFI_PREAMBLE_LONG;
    txInfo.channelNumber = CCH;
    txInfo.dataRate = WifiMode("OfdmRate12MbpsBW10MHz");
    txInfo.priority = 3;
    txInfo.txPowerLevel = 3;

    // Create packet and send as broadcast
    Ptr<Packet> packet = Create<Packet>(msgBuffer.get(), msgSize);
    Mac48Address broadcastAddress = Mac48Address::GetBroadcast();
    uint16_t protocolId = 0x88dc;

    netDevice->SendX(packet, broadcastAddress, protocolId, txInfo);
}


// device is the receiver here, neighbourMac is the sender of the packet
void
Update_Warning(Ptr<WaveNetDevice> device,
                  Mac48Address neighborMac,
                  double neighborX,
                  double neighborY)
{
    Mac48Address selfMac = Mac48Address::ConvertFrom(device->GetAddress());
    Vector selfPosition = device->GetNode()->GetObject<MobilityModel>()->GetPosition();

    //self mac is receiver, neighbor mac is sender
    // neighborX and neighborY are the coordinates of the sender

    bool neighborFound = false;
    double dx = selfPosition.x - neighborX;
    double dy = selfPosition.y - neighborY;
    double distance = std::sqrt(dx * dx + dy * dy);

    auto& neighbors = neighborTable[selfMac];
    for (auto& neighbor : neighbors)
    {
        if (neighbor.mac == neighborMac)
        {
            neighborFound = true;
            neighbor.distance = distance;

            if (distance < WARNING_DISTANCE)
            {
                std::cout << "Collision Warning Packet sent at time " << Simulator::Now().GetSeconds() << " seconds." << std::endl;

                std::cout << "Collision Warning: Vehicle " << selfMac
                          << " is dangerously close to Vehicle " << neighborMac
                          << ". Reduce the Speed" << std::endl;
                
                // back to the sender, send a collision warning packet from the receiver   
                SendCollisionWarning(device, neighborMac);

                // now we send a collision warning packet to the sender
            }

            break;
        }
    }

    if (!neighborFound)
    {
        if (distance < WARNING_DISTANCE)
        {
            std::cout << "Collision Warning Packet sent at time " << Simulator::Now().GetSeconds() << " seconds." << std::endl;

            std::cout << "Collision Warning: Vehicle " << selfMac
                      << " is dangerously close to Vehicle " << neighborMac
                      << ". Reduce the Speed" << std::endl;
            
            // back to the sender, send a collision warning packet from the receiver   
            SendCollisionWarning(device, neighborMac);
        }
        
        neighbors.push_back({neighborMac, distance});
    }
}

bool
ReceivePacket(Ptr<NetDevice> device,
              Ptr<const Packet> packet,
              uint16_t protocol,
              const Address& sender)
{

    // device is reciever here, sender is the sender of the packet

    Mac48Address destMac = Mac48Address::ConvertFrom(device->GetAddress());
    Mac48Address srcMac = Mac48Address::ConvertFrom(sender);

    std::cout << TEAL_CODE << BOLD_CODE << "Received packet at Vehicle " << destMac
              << " from Vehicle " << srcMac
              << " at time " << Simulator::Now().GetSeconds() << " seconds." << END_CODE << std::endl;


    // Copy and extract packet contents
    Ptr<Packet> packetCopy = packet->Copy();
    uint32_t packetSize = packetCopy->GetSize();
    uint8_t* buffer = new uint8_t[packetSize];
    packetCopy->CopyData(buffer, packetSize);
    std::string message(reinterpret_cast<char*>(buffer), packetSize);
    delete[] buffer;

    // Print message with appropriate color
    if (message.find("Collision Warning") != std::string::npos)
    {
        std::cout << RED_CODE << BOLD_CODE << "Received Collision Warning: " << message << END_CODE << std::endl;
        return true; // Collision warnings do not trigger further processing
    }
    else if (message.find("Hello from vehicle") != std::string::npos)
    {
        std::cout << YELLOW_CODE << BOLD_CODE << "Received Hello Message: " << message << END_CODE << std::endl;
    }
    else
    {
        std::cout << "Received message: " << message << std::endl;
    }

    // Extract coordinates
    size_t xPos = message.find("x:");
    size_t yPos = message.find("y:");

    if (xPos != std::string::npos && yPos != std::string::npos)
    {
        std::string xStr = message.substr(xPos + 2, yPos - (xPos + 2));
        std::string yStr = message.substr(yPos + 2);

        try
        {
            double x = std::stod(xStr);
            double y = std::stod(yStr);

            Ptr<WaveNetDevice> myDevice = DynamicCast<WaveNetDevice>(device);
            Update_Warning(myDevice, srcMac, x, y);

            // receiver, sender, x, y
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing coordinates: " << e.what() << std::endl;
        }
    }
    else
    {
        std::cerr << "Invalid message format (missing x: or y:)" << std::endl;
    }

    return true;
}


// Function to simulate the environment and print statuses every second
void
SimulateEnvironment(NodeContainer& vehicles)
{
    // Print vehicle locations and neighbor tables every second
    PrintVehicleLocations(vehicles);
    PrintNeighborTable(vehicles);

    // Schedule the next simulation step
    Simulator::Schedule(Seconds(1), &SimulateEnvironment, vehicles);
}


// Main function to set up the simulation
int
main(int argc, char* argv[])
{
    CommandLine cmd;
    uint32_t nNodes = 3;
    cmd.AddValue("n", "Number of vehicles", nNodes);
    cmd.Parse(argc, argv);

    double simTime = 5;
    NodeContainer vehicles;
    vehicles.Create(nNodes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds",
                              RectangleValue(Rectangle(-100, 100, -100, 100)),
                              "Speed",
                              StringValue("ns3::UniformRandomVariable[Min=5.0|Max=15.0]"));
    mobility.Install(vehicles);

    YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
    YansWavePhyHelper wavePhy = YansWavePhyHelper::Default();
    wavePhy.SetChannel(waveChannel.Create());

    QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
    WaveHelper waveHelper = WaveHelper::Default();
    waveHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode",
                                       StringValue("OfdmRate6MbpsBW10MHz"),
                                       "ControlMode",
                                       StringValue("OfdmRate6MbpsBW10MHz"),
                                       "NonUnicastMode",
                                       StringValue("OfdmRate6MbpsBW10MHz"));

    NetDeviceContainer devices = waveHelper.Install(wavePhy, waveMac, vehicles);

    for (uint32_t i = 0; i < nNodes; i++)
    {
        Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(devices.Get(i));
        Mac48Address Mac = Mac48Address::ConvertFrom(device->GetAddress());
        neighborTable[Mac] = {}; 

    }

    for (uint32_t i = 0; i < devices.GetN(); ++i)
    {
        Ptr<NetDevice> dev = devices.Get(i);
        dev->SetReceiveCallback(MakeCallback(&ReceivePacket));
    }

    for (int time = 1; time <= simTime; time++)
    {
        for (uint32_t i = 0; i < nNodes; i++)
        {
            Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(devices.Get(i));

            // Start Hello packet broadcasting
            Simulator::Schedule(Seconds(time + 0.01 * i), &SendHello, device, i);
        }
    }

    // Start the simulation environment printout
    Simulator::Schedule(Seconds(1), &SimulateEnvironment, vehicles);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
}