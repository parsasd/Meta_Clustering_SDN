#include <ns3/core-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-apps-module.h>
#include <ns3/internet-module.h>
#include <ns3/network-module.h>
#include <ns3/ofswitch13-module.h>
#include <ns3/mobility-module.h>
#include <ns3/log.h>
#include "qvtab.h"
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <unordered_map>
#include <chrono>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MetaLearnSimulation");

uint32_t packetsSent = 0;
uint32_t packetsReceived = 0;
double totalEndToEndDelay = 0.0;
uint32_t clusterHeadChanges = 0;
uint32_t totalClusterTransitions = 0;


std::ofstream clusterLog("cluster_head_lifetimes.csv", std::ios::out);
std::ofstream packetLog("packet_metrics.csv", std::ios::out);

// Initialize logs
void InitializeLogs() {
    clusterLog << "NodeID,ClusterHead,ClusterHeadLifetime,Time\n";
    packetLog << "PacketID,Source,Destination,Delay\n";
}

// Placeholder for packet routing
void RoutePacket(Ptr<Node> sourceNode, Ptr<Node> destinationNode, QVTab &qvTab) {
    packetsSent++;
    double delay = rand() % 5; // Simulated delay
    totalEndToEndDelay += delay;
    packetsReceived++;

    // Log packet delivery
    packetLog << packetsSent << ","
              << sourceNode->GetId() << ","
              << destinationNode->GetId() << ","
              << delay << "\n";
}

void RoutePacketPeriodically(Ptr<Node> sourceNode, Ptr<Node> destinationNode, QVTab &qvTab, Time interval) {
    RoutePacket(sourceNode, destinationNode, qvTab);

    // Re-schedule packet routing
    Simulator::Schedule(interval, &RoutePacketPeriodically, sourceNode, destinationNode, std::ref(qvTab), interval);
}

void PrintMetrics() {
    NS_LOG_INFO("Packets Sent: " << packetsSent);
    NS_LOG_INFO("Packets Received: " << packetsReceived);
    NS_LOG_INFO("Total End-to-End Delay: " << totalEndToEndDelay);
}

struct Wolf {
  Ptr<Node> node;
  double fitness;
  QVTab qvTab;  // Each wolf will have its QVTab
  Ptr<Node> clusterHead; // Pointer to the current cluster head
  Time lastClusterChange; // Use NS-3 Time instead of std::chrono::time_point
  Time clusterHeadStartTime; // Use NS-3 Time instead of std::chrono::time_point
  double clusterHeadLifetime = 0.0; // Total time spent as cluster head
  bool isClusterHead = false; // Flag to indicate if the node is currently a cluster head
};

// Metrics

// Threshold for fitness difference to initiate cluster change
const double FITNESS_THRESHOLD = 0.1;
// Minimum time between cluster changes for a node (in milliseconds)
const double MIN_CLUSTER_CHANGE_INTERVAL_S = 5.0;

// Declare RSU nodes globally to be accessible in all functions
NodeContainer rsus;

// Helper functions for vector operations
Vector MultiplyVector(const Vector &v, double scalar) {
  return Vector(v.x * scalar, v.y * scalar, v.z * scalar);
}

Vector SubtractVector(const Vector &v1, const Vector &v2) {
  return Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

Vector AddVector(const Vector &v1, const Vector &v2) {
  return Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

Vector DivideVector(const Vector &v, double scalar) {
  return Vector(v.x / scalar, v.y / scalar, v.z / scalar);
}

// Function to calculate the fitness value of a vehicle node
double CalculateFitness(Ptr<Node> node) {
  double linkQuality = static_cast<double>(rand() % 100) / 100.0;  // Simulated link quality (0-1)
  double velocity = node->GetObject<MobilityModel>()->GetVelocity().GetLength();  // Get current speed of the vehicle
  double normalizedVelocity = (velocity > 0) ? (1.0 / (1.0 + velocity)) : 1.0;  // Normalize velocity (lower speed is better)
  double packetDropRate = static_cast<double>(rand() % 100) / 100.0;  // Simulated packet drop rate (0-1)

  // Fitness is calculated considering link quality, velocity, and packet drop rate
  double fitness = (0.5 * linkQuality) + (0.3 * normalizedVelocity) + (0.2 * (1 - packetDropRate));
  return fitness;
}

void PerformGWO(std::vector<Wolf> &wolves, int iterations) {
  for (int iter = 0; iter < iterations; ++iter) {
    // Sort wolves based on fitness value
    std::sort(wolves.begin(), wolves.end(), [](const Wolf &a, const Wolf &b) {
      return a.fitness > b.fitness;
    });

    // Select alpha, beta, and delta wolves (top 3)
    Wolf alpha = wolves[0];
    Wolf beta = wolves[1];
    Wolf delta = wolves[2];

    // Ensure alpha, beta, and delta are cluster heads
    wolves[0].isClusterHead = true;
    wolves[1].isClusterHead = true;
    wolves[2].isClusterHead = true;
    wolves[0].clusterHeadStartTime = Simulator::Now();
    wolves[1].clusterHeadStartTime = Simulator::Now();
    wolves[2].clusterHeadStartTime = Simulator::Now();

// for (const auto &wolf : wolves) {
//      NS_LOG_INFO("Node " << wolf.node->GetId() << "cluster head: " << wolf.isClusterHead);
//     if (wolf.isClusterHead) {
//       NS_LOG_INFO("Node " << wolf.node->GetId() 
//                          << " Final Cluster Head Lifetime: " 
//                          << wolf.clusterHeadLifetime << " seconds");
//     }
//   }
    // Update the position of the remaining wolves based on alpha, beta, and delta
    for (size_t i = 3; i < wolves.size(); ++i) {
      Wolf &wolf = wolves[i];
      Vector alphaPos = alpha.node->GetObject<MobilityModel>()->GetPosition();
      Vector betaPos = beta.node->GetObject<MobilityModel>()->GetPosition();
      Vector deltaPos = delta.node->GetObject<MobilityModel>()->GetPosition();
      Vector wolfPos = wolf.node->GetObject<MobilityModel>()->GetPosition();

      // Coefficients A and C
      double a = 2.0 - (2.0 * iter / iterations);
      double r1 = static_cast<double>(rand() % 100) / 100.0;
      double r2 = static_cast<double>(rand() % 100) / 100.0;
      double A_scalar = 2 * a * r1 - a;
      double C_scalar = 2 * r2;

      // Adaptive coefficients for exploration and exploitation
      double deltaAdaptive = static_cast<double>(rand() % 100) / 100.0;
      double cTAdaptive = static_cast<double>(rand() % 100) / 100.0;
      if (deltaAdaptive > 1 && cTAdaptive > 1) {
        A_scalar *= 1.5;  // Increase A to encourage exploration
        C_scalar *= 1.5;  // Increase C to explore more aggressively
      } else if (deltaAdaptive < 1 && cTAdaptive < 1) {
        A_scalar *= 0.5;  // Decrease A to encourage exploitation
        C_scalar *= 0.5;  // Decrease C to exploit current area
      }

      // Calculate D_alpha, D_beta, D_delta
      Vector D_alpha = MultiplyVector(SubtractVector(alphaPos, wolfPos), C_scalar);
      Vector D_beta = MultiplyVector(SubtractVector(betaPos, wolfPos), C_scalar);
      Vector D_delta = MultiplyVector(SubtractVector(deltaPos, wolfPos), C_scalar);

      // Update position using scalar multiplications
      Vector newAlphaPos = SubtractVector(alphaPos, MultiplyVector(D_alpha, A_scalar));
      Vector newBetaPos = SubtractVector(betaPos, MultiplyVector(D_beta, A_scalar));
      Vector newDeltaPos = SubtractVector(deltaPos, MultiplyVector(D_delta, A_scalar));

      // Calculate the new position as the average
      Vector newPos = DivideVector(AddVector(AddVector(newAlphaPos, newBetaPos), newDeltaPos), 3);
      wolf.node->GetObject<MobilityModel>()->SetPosition(newPos);

      // Recalculate fitness after updating position
      wolf.fitness = CalculateFitness(wolf.node);
    }
    clusterHeadChanges++;  // Increment cluster head changes
  }
}

// Beacon message broadcast function
void BroadcastBeacon(Ptr<Node> node, QVTab &qvTab, std::vector<Wolf> &wolves) {
  Time now = Simulator::Now(); // Use Simulator::Now()
  double qValue = CalculateFitness(node);  // Calculate current Q-value
  qvTab.UpdateQValue(node->GetId(), qValue, node->GetObject<MobilityModel>()->GetPosition());

  for (auto &wolf : wolves) {
    if (wolf.node == node) {
      continue;  // Skip self
    }
    double neighborFitness = CalculateFitness(wolf.node);

    // Time since last cluster change (in seconds)
    double timeSinceLastChange = (now - wolf.lastClusterChange).GetSeconds();
    // NS_LOG_INFO("Node " << wolf.node->GetId() 
    //          << " lastClusterChange: " 
    //          << wolf.lastClusterChange.GetSeconds()
    //          << " seconds since epoch.");
    // NS_LOG_INFO("now "
    //          << now.GetSeconds()
    //          << " seconds since epoch.");

    // NS_LOG_INFO("Node " << wolf.node->GetId() << " Time Since Last Change: " << timeSinceLastChange << " seconds");

    if ((neighborFitness > qValue + FITNESS_THRESHOLD) &&
        (!wolf.clusterHead || wolf.clusterHead->GetId() != node->GetId()) &&
        timeSinceLastChange >= MIN_CLUSTER_CHANGE_INTERVAL_S) {
      wolf.clusterHead = node;  // Update cluster head
      wolf.lastClusterChange = now;
      totalClusterTransitions++;  // Increment total cluster transitions
      wolf.clusterHeadLifetime = (now - wolf.clusterHeadStartTime).GetSeconds();

    }
  }
}

void FinalizeClusterHeadLifetime(std::vector<Wolf> &wolves) {
    Time now = Simulator::Now();
    for (auto &wolf : wolves) {
        if (wolf.isClusterHead) {
            double lifetime = (now - wolf.clusterHeadStartTime).GetSeconds();
            wolf.clusterHeadLifetime += lifetime;

            // Log cluster head lifetime
            clusterLog << wolf.node->GetId() << ","
                       << (wolf.clusterHead ? wolf.clusterHead->GetId() : -1) << ","
                       << wolf.clusterHeadLifetime << ","
                       << now.GetSeconds() << "\n";
        }
    }
}


void PrintClusterHeadLifetimes(std::vector<Wolf> &wolves) {
  for (const auto &wolf : wolves) {
    //  NS_LOG_INFO("Node " << wolf.node->GetId() << "cluster head: " << wolf.isClusterHead);
    if (wolf.isClusterHead) {
      NS_LOG_INFO("Node " << wolf.node->GetId() 
                         << " Final Cluster Head Lifetime: " 
                         << wolf.clusterHeadLifetime << " seconds");
    }
  }
}


void BroadcastBeaconPeriodically(Ptr<Node> node, QVTab &qvTab, std::vector<Wolf> &wolves, Time interval) {
  BroadcastBeacon(node, qvTab, wolves);
// NS_LOG_INFO("Current simulation time: " << Simulator::Now().GetSeconds() << " seconds");

  // Re-schedule the beacon broadcast
  Simulator::Schedule(interval, &BroadcastBeaconPeriodically, node, std::ref(qvTab), std::ref(wolves), interval);
}

void PerformGWOPeriodically(std::vector<Wolf> &wolves, int iterations, Time interval) {
  NS_LOG_INFO("Performing GWO optimization...");

  // Call the PerformGWO function to execute the optimization logic
  PerformGWO(wolves, iterations);

  // Schedule the next GWO execution
  Simulator::Schedule(interval, &PerformGWOPeriodically, std::ref(wolves), iterations, interval);
}

int main(int argc, char *argv[]) {
  // Enable logging and checksum computation
  LogComponentEnable("MetaLearnSimulation", LOG_LEVEL_INFO);
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  // Create vehicle nodes
  NodeContainer vehicles;
  vehicles.Create(25);

  // Create RSU nodes
  rsus.Create(4);

  InitializeLogs();

  // Create the switch node
  Ptr<Node> switchNode = CreateObject<Node>();

  // Connect vehicles and RSUs to the switch using CSMA links
  CsmaHelper csmaHelper;
  NetDeviceContainer vehicleDevices, switchPorts, rsuDevices;
  for (size_t i = 0; i < vehicles.GetN(); ++i) {
    NodeContainer pair(vehicles.Get(i), switchNode);
    NetDeviceContainer link = csmaHelper.Install(pair);
    vehicleDevices.Add(link.Get(0));
    switchPorts.Add(link.Get(1));
  }
  for (size_t i = 0; i < rsus.GetN(); ++i) {
    NodeContainer pair(rsus.Get(i), switchNode);
    NetDeviceContainer link = csmaHelper.Install(pair);
    rsuDevices.Add(link.Get(0));
    switchPorts.Add(link.Get(1));
  }

  // Set up mobility for vehicles
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
    double x = (i % 5) * 20.0; // Grid layout
    double y = (i / 5) * 20.0;
    positionAlloc->Add(Vector(x, y, 0));
  }
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=20.0]"),
                            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                            "PositionAllocator", PointerValue(positionAlloc));
  mobility.Install(vehicles);

  // Set up mobility for RSUs
  MobilityHelper rsuMobility;
  Ptr<ListPositionAllocator> rsuPositionAlloc = CreateObject<ListPositionAllocator>();
  rsuPositionAlloc->Add(Vector(50, 50, 0));
  rsuPositionAlloc->Add(Vector(150, 50, 0));
  rsuPositionAlloc->Add(Vector(50, 150, 0));
  rsuPositionAlloc->Add(Vector(150, 150, 0));
  rsuMobility.SetPositionAllocator(rsuPositionAlloc);
  rsuMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  rsuMobility.Install(rsus);

  // Set up the controller and OpenFlow network domain
  Ptr<Node> controllerNode = CreateObject<Node>();
  Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper>();
  of13Helper->InstallController(controllerNode);
  of13Helper->InstallSwitch(switchNode, switchPorts);
  of13Helper->CreateOpenFlowChannels();

  // Initialize wolves
  std::vector<Wolf> wolves;
 for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
  Wolf wolf;
  wolf.node = vehicles.Get(i);
  wolf.fitness = CalculateFitness(wolf.node);
  wolf.lastClusterChange = Simulator::Now(); // Initialize to "zero"
  wolf.clusterHeadStartTime = Simulator::Now(); // Start as the current time
  wolves.push_back(wolf);
}


  // Schedule periodic GWO
   // Schedule periodic operations
    Simulator::Schedule(Seconds(10.0), &PerformGWOPeriodically, std::ref(wolves), 10, Seconds(10.0));
    for (auto &wolf : wolves) {
        Simulator::Schedule(Seconds(5.0), &BroadcastBeaconPeriodically, wolf.node, std::ref(wolf.qvTab), std::ref(wolves), Seconds(5.0));
    }
    Ptr<UniformRandomVariable> randomVar = CreateObject<UniformRandomVariable>();
    for (size_t i = 0; i < wolves.size() / 2; ++i) {
        uint32_t srcIndex = randomVar->GetInteger(0, wolves.size() - 1);
        uint32_t dstIndex = randomVar->GetInteger(0, wolves.size() - 1);
        while (dstIndex == srcIndex) {
            dstIndex = randomVar->GetInteger(0, wolves.size() - 1);
        }
        Simulator::Schedule(Seconds(15.0), &RoutePacketPeriodically, wolves[srcIndex].node, wolves[dstIndex].node, std::ref(wolves[srcIndex].qvTab), Seconds(15.0));
    }

  Simulator::Schedule(Seconds(99.0), &FinalizeClusterHeadLifetime, std::ref(wolves));
  Simulator::Schedule(Seconds(100.0), &PrintMetrics);
  Simulator::Stop(Seconds(100.0));

  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
