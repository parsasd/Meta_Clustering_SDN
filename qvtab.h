#ifndef QVTAB_H
#define QVTAB_H

#include <map>
#include <ns3/vector.h>
#include <ns3/node.h>

class QVTab {
public:
  void UpdateQValue(uint32_t nodeId, double qValue, ns3::Vector position);
  void UpdateEligibilityTrace(uint32_t nodeId, double traceValue);
  void UpdateQValuesWithTD(double reward, double gamma, double alpha);

  double GetQValue(uint32_t nodeId);
  double GetEligibilityTrace(uint32_t nodeId);
  std::map<uint32_t, std::pair<double, ns3::Vector>> GetQTable();
  void PrintQTable();

private:
  struct NodeInfo {
    double qValue;
    ns3::Vector position;
    double eligibilityTrace;
  };

  std::map<uint32_t, NodeInfo> qTable;
};

#endif // QVTAB_H
