#include "qvtab.h"
#include <iostream>

using namespace ns3;

void QVTab::UpdateQValue(uint32_t nodeId, double qValue, Vector position) {
  qTable[nodeId].qValue = qValue;
  qTable[nodeId].position = position;
}

void QVTab::UpdateEligibilityTrace(uint32_t nodeId, double traceValue) {
  qTable[nodeId].eligibilityTrace = traceValue;
}

double QVTab::GetQValue(uint32_t nodeId) {
  return qTable.find(nodeId) != qTable.end() ? qTable[nodeId].qValue : 0.0;
}

double QVTab::GetEligibilityTrace(uint32_t nodeId) {
  return qTable.find(nodeId) != qTable.end() ? qTable[nodeId].eligibilityTrace : 0.0;
}

std::map<uint32_t, std::pair<double, Vector>> QVTab::GetQTable() {
  std::map<uint32_t, std::pair<double, Vector>> qTableCopy;
  for (auto &entry : qTable) {
    qTableCopy[entry.first] = {entry.second.qValue, entry.second.position};
  }
  return qTableCopy;
}

void QVTab::UpdateQValuesWithTD(double reward, double gamma, double alpha) {
  for (auto &entry : qTable) {
    double tdError = reward + gamma * entry.second.qValue - entry.second.qValue;
    entry.second.qValue += alpha * tdError * entry.second.eligibilityTrace;
    entry.second.eligibilityTrace *= gamma * alpha; // Decay
  }
}

void QVTab::PrintQTable() {
  for (const auto &entry : qTable) {
    std::cout << "Node: " << entry.first
              << ", Q-Value: " << entry.second.qValue
              << ", Position: (" << entry.second.position.x << ", " << entry.second.position.y << ", " << entry.second.position.z << ")"
              << ", Eligibility Trace: " << entry.second.eligibilityTrace << std::endl;
  }
}
