#pragma once

#include <pthread.h>
#include <sched.h>
#include <optional>
#include <system_error>

// Used only by the opt-in Linux benchmark, never by the production solve path.
class BenchThreadConfiguration {
  struct Request {
    std::optional<std::vector<int>> cpus;
    std::optional<int> policy;
    int priority = 0;
  };
  const char* names[3]{"caller","newton","nlopt"};
  std::array<Request,3> requests;
  std::array<PIN_IK::WorkerThreadSettings,3> original, effective;
  bool caller_saved = false, restored = false;

  static void check(int error, const char* operation) {
    if (error) throw std::system_error(error,std::generic_category(),operation);
  }
  static int integer(const std::string& value) {
    size_t used=0;
    const int result=std::stoi(value,&used);
    if (used!=value.size()) throw std::invalid_argument("Invalid integer: "+value);
    return result;
  }
  static PIN_IK::WorkerThreadSettings callerSettings() {
    PIN_IK::WorkerThreadSettings state;
    cpu_set_t mask; CPU_ZERO(&mask);
    sched_param parameters{};
    check(pthread_getaffinity_np(pthread_self(),sizeof(mask),&mask),"get caller affinity");
    check(pthread_getschedparam(pthread_self(),&state.policy,&parameters),"get caller scheduling");
    state.priority=parameters.sched_priority;
    for (int i=0;i<CPU_SETSIZE;++i) if (CPU_ISSET(i,&mask)) state.cpus.push_back(i);
    return state;
  }
  static int callerAffinity(const std::vector<int>& cpus) {
    cpu_set_t mask; CPU_ZERO(&mask);
    for (int cpu:cpus) CPU_SET(cpu,&mask);
    return pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask);
  }
  static int callerScheduling(int policy,int priority) {
    sched_param parameters{}; parameters.sched_priority=priority;
    return pthread_setschedparam(pthread_self(),policy,&parameters);
  }
  static void stateJSON(std::ostream& out,const PIN_IK::WorkerThreadSettings& state) {
    out << "{\"cpus\":[";
    for (size_t i=0;i<state.cpus.size();++i) out << (i ? "," : "") << state.cpus[i];
    out << "],\"policy\":" << state.policy << ",\"priority\":" << state.priority << '}';
  }
public:
  explicit BenchThreadConfiguration(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read thread configuration: "+path);
    std::array<bool,3> seen{};
    std::string line;
    while (std::getline(input,line)) {
      line=line.substr(0,line.find('#'));
      std::istringstream fields(line);
      std::string role,cpus,policy,priority,extra;
      if (!(fields>>role)) continue;
      if (!(fields>>cpus>>policy>>priority) || fields>>extra)
        throw std::invalid_argument("Expected: role cpus policy priority");
      unsigned index=0;
      while (index<3 && role!=names[index]) ++index;
      if (index==3 || seen[index]) throw std::invalid_argument("Unknown/duplicate role: "+role);
      seen[index]=true;
      auto& request=requests[index];
      if (cpus!="-") {
        request.cpus.emplace();
        if (cpus.back()==',') throw std::invalid_argument("Trailing comma in CPU set");
        std::istringstream list(cpus); std::string cpu;
        while (std::getline(list,cpu,',')) {
          int id=integer(cpu);
          if (id<0 || id>=CPU_SETSIZE) throw std::invalid_argument("CPU ID out of range");
          request.cpus->push_back(id);
        }
        std::sort(request.cpus->begin(),request.cpus->end());
        request.cpus->erase(std::unique(request.cpus->begin(),request.cpus->end()),request.cpus->end());
      }
      if (policy=="keep") {
        if (priority!="-") throw std::invalid_argument("keep requires priority '-'");
      } else {
        if (policy=="other") request.policy=SCHED_OTHER;
        else if (policy=="fifo") request.policy=SCHED_FIFO;
        else if (policy=="rr") request.policy=SCHED_RR;
        else throw std::invalid_argument("Unknown policy: "+policy);
        request.priority=integer(priority);
        if (request.priority<sched_get_priority_min(*request.policy) ||
            request.priority>sched_get_priority_max(*request.policy))
          throw std::invalid_argument("Priority out of range");
      }
    }
    if (!std::all_of(seen.begin(),seen.end(),[](bool value){return value;}))
      throw std::invalid_argument("Configuration must contain caller, newton and nlopt");
  }
  BenchThreadConfiguration(const BenchThreadConfiguration&)=delete;
  BenchThreadConfiguration& operator=(const BenchThreadConfiguration&)=delete;
  ~BenchThreadConfiguration() {
    if (caller_saved && !restored) {
      // Best effort during exception unwinding; normal completion uses checked restore().
      const int scheduling=callerScheduling(original[0].policy,original[0].priority);
      const int affinity=callerAffinity(original[0].cpus);
      if (scheduling || affinity) std::cerr << "Caller restore failed: scheduling=" << scheduling << " affinity=" << affinity << '\n';
    }
  }
  void apply(PIN_IK::PIN_IK& solver) {
    original[0]=callerSettings(); caller_saved=true;
    original[1]=solver.getWorkerThreadSettings(PIN_IK::WorkerThread::Newton);
    original[2]=solver.getWorkerThreadSettings(PIN_IK::WorkerThread::Nlopt);
    // Configure workers before elevating the caller. No process-wide settings.
    for (unsigned index : {1U,2U,0U}) {
      const auto& request=requests[index];
      const auto worker=static_cast<PIN_IK::WorkerThread>(index ? index-1 : 0);
      try {
        if (request.cpus) {
          if (index) solver.setWorkerThreadAffinity(worker,*request.cpus);
          else check(callerAffinity(*request.cpus),"set caller affinity");
        }
        if (request.policy) {
          if (index) solver.setWorkerThreadScheduling(worker,*request.policy,request.priority);
          else check(callerScheduling(*request.policy,request.priority),"set caller scheduling");
        }
        effective[index]=index ? solver.getWorkerThreadSettings(worker) : callerSettings();
        if ((request.cpus && *request.cpus!=effective[index].cpus) ||
            (request.policy && (*request.policy!=effective[index].policy || request.priority!=effective[index].priority)))
          throw std::runtime_error(std::string(names[index])+": requested settings differ from OS readback");
      } catch (const std::system_error& e) {
        throw std::system_error(e.code(),std::string(names[index])+": "+e.what());
      }
    }
  }
  void restore() {
    check(callerScheduling(original[0].policy,original[0].priority),"restore caller scheduling");
    check(callerAffinity(original[0].cpus),"restore caller affinity");
    const auto actual=callerSettings();
    if (actual.cpus!=original[0].cpus || actual.policy!=original[0].policy || actual.priority!=original[0].priority)
      throw std::runtime_error("Caller restore readback mismatch");
    restored=true;
  }
  void write(const std::string& path) const {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Cannot write configuration metadata");
    output << "{\"caller_restored\":" << (restored ? "true" : "false") << ",\"threads\":{";
    for (unsigned i=0;i<3;++i) {
      if (i) output << ',';
      output << '"' << names[i] << "\":{\"original\":";
      stateJSON(output,original[i]); output << ",\"effective\":"; stateJSON(output,effective[i]);
      output << ",\"requested\":{\"cpus\":";
      if (!requests[i].cpus) output << "null";
      else {
        output << '[';
        for (size_t j=0;j<requests[i].cpus->size();++j) output << (j ? "," : "") << (*requests[i].cpus)[j];
        output << ']';
      }
      output << ",\"policy\":";
      if (requests[i].policy) output << *requests[i].policy; else output << "null";
      output << ",\"priority\":" << requests[i].priority << "}}";
    }
    output << "}}\n";
    if (!output) throw std::runtime_error("Configuration metadata write failed");
  }
};
