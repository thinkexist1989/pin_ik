#include <pin_ik/pin_ik.hpp>
#include <fstream>
#include <iostream>
#include <system_error>
#include <cerrno>

#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <linux/capability.h>
#include <unistd.h>
namespace PIN_IK {
struct WorkerTestAccess {
  static std::thread& thread(PIN_IK& solver, unsigned index) {
    return index ? solver.task2 : solver.task1;
  }
};
}
#endif

using Worker = PIN_IK::WorkerThread;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
bool equal(const PIN_IK::WorkerThreadSettings& a, const PIN_IK::WorkerThreadSettings& b) {
  return a.cpus == b.cpus && a.policy == b.policy && a.priority == b.priority;
}
template<class F> void invalid(F function) {
  try { function(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("Invalid configuration was accepted");
}

int main(int argc, char** argv) {
  try {
    require(argc >= 2 && argc <= 3, "Expected URDF [--realtime|--deny-realtime]");
    const std::string mode = argc == 3 ? argv[2] : "";
    require(mode.empty() || mode == "--realtime" || mode == "--deny-realtime", "Invalid mode");
#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
    if (mode == "--deny-realtime") {
      __user_cap_header_struct header{_LINUX_CAPABILITY_VERSION_3,0};
      __user_cap_data_struct capabilities[2]{};
      require(syscall(SYS_capget,&header,capabilities) == 0,"capget failed");
      if (capabilities[CAP_SYS_NICE/32].effective & (1U << (CAP_SYS_NICE%32))) {
        std::cout << "SKIP: CAP_SYS_NICE bypasses RLIMIT_RTPRIO\n"; return 77;
      }
      sched_param ordinary{};
      require(pthread_setschedparam(pthread_self(),SCHED_OTHER,&ordinary)==0,"Cannot normalize permission-test caller");
      const rlimit limit{0,0};
      require(setrlimit(RLIMIT_RTPRIO,&limit) == 0,"Cannot restrict own RTPRIO limit");
    }
#endif
    std::ifstream input(argv[1]);
    require(bool(input),"Cannot read URDF");
    const std::string xml((std::istreambuf_iterator<char>(input)),{});
    PIN_IK::PIN_IK solver("base","tip",xml,.02,1e-6);
    const Worker workers[]{Worker::Newton,Worker::Nlopt};
#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
    const auto first = solver.getWorkerThreadSettings(workers[0]);
    const auto second = solver.getWorkerThreadSettings(workers[1]);
    require(!first.cpus.empty() && !second.cpus.empty(),"Empty initial CPU mask");
    const auto ids = std::array<std::thread::id,2>{
      PIN_IK::WorkerTestAccess::thread(solver,0).get_id(),
      PIN_IK::WorkerTestAccess::thread(solver,1).get_id()};
    cpu_set_t caller_before,caller_after;
    require(pthread_getaffinity_np(pthread_self(),sizeof(caller_before),&caller_before)==0,"Caller affinity");
    int caller_policy; sched_param caller_parameters{};
    require(pthread_getschedparam(pthread_self(),&caller_policy,&caller_parameters)==0,"Caller scheduler");
    solver.setWorkerThreadAffinity(workers[0],{first.cpus.front(),first.cpus.front()});
    solver.setWorkerThreadAffinity(workers[1],{second.cpus.back()});
    for (unsigned i=0;i<2;++i) {
      auto state = solver.getWorkerThreadSettings(workers[i]);
      require(state.cpus == std::vector<int>{i ? second.cpus.back() : first.cpus.front()},"Affinity not applied independently");
      cpu_set_t native;
      int policy; sched_param parameters{};
      auto handle = PIN_IK::WorkerTestAccess::thread(solver,i).native_handle();
      require(pthread_getaffinity_np(handle,sizeof(native),&native)==0,"Native affinity read");
      require(CPU_COUNT(&native)==1 && CPU_ISSET(state.cpus.front(),&native),"Native affinity mismatch");
      require(pthread_getschedparam(handle,&policy,&parameters)==0,"Native scheduling read");
      require(state.policy==policy && state.priority==parameters.sched_priority,"Native scheduler mismatch");
      invalid([&]{solver.setWorkerThreadAffinity(workers[i],{});});
      invalid([&]{solver.setWorkerThreadAffinity(workers[i],{-1});});
      invalid([&]{solver.setWorkerThreadAffinity(workers[i],{CPU_SETSIZE});});
      invalid([&]{solver.setWorkerThreadScheduling(workers[i],-1,0);});
      invalid([&]{solver.setWorkerThreadScheduling(workers[i],SCHED_OTHER,1);});
      require(equal(state,solver.getWorkerThreadSettings(workers[i])),"Invalid request changed state");
      // A syntactically valid mask with no CPU on this machine exercises an
      // actual pthread_setaffinity_np failure, rather than input validation.
      std::ifstream last_cpu("/sys/devices/system/cpu/cpu"+std::to_string(CPU_SETSIZE-1)+"/topology/core_id");
      if (!last_cpu) {
        bool rejected=false;
        try { solver.setWorkerThreadAffinity(workers[i],{CPU_SETSIZE-1}); }
        catch (const std::system_error& e) { require(e.code().value()==EINVAL,"Unexpected affinity error"); rejected=true; }
        require(rejected && equal(state,solver.getWorkerThreadSettings(workers[i])),"Kernel rejection changed affinity");
      }
    }
    invalid([&]{solver.getWorkerThreadSettings(static_cast<Worker>(2));});
    invalid([&]{solver.setWorkerThreadAffinity(static_cast<Worker>(-1),{0});});
    invalid([&]{solver.setWorkerThreadScheduling(static_cast<Worker>(2),SCHED_OTHER,0);});
    if (mode == "--deny-realtime") {
      for (auto worker : workers) {
        const auto before = solver.getWorkerThreadSettings(worker);
        bool denied=false;
        try { solver.setWorkerThreadScheduling(worker,SCHED_FIFO,1); }
        catch (const std::system_error& e) { require(e.code().value()==EPERM,"Unexpected scheduling error"); denied=true; }
        require(denied,"Expected real kernel EPERM");
        require(equal(before,solver.getWorkerThreadSettings(worker)),"EPERM changed settings");
      }
      std::cout << "Real kernel EPERM reported; previous explicit affinity retained\n";
    }
    if (mode == "--realtime") {
      for (int policy : {SCHED_FIFO,SCHED_RR}) for (auto worker : workers) {
        try { solver.setWorkerThreadScheduling(worker,policy,1); }
        catch (const std::system_error& e) {
          if (e.code().value()==EPERM || e.code().value()==EACCES) {
            std::cout << "SKIP: realtime scheduling denied: " << e.what() << '\n'; return 77;
          }
          throw;
        }
        auto state=solver.getWorkerThreadSettings(worker);
        require(state.policy==policy && state.priority==1,"Realtime scheduling readback mismatch");
        solver.setWorkerThreadScheduling(worker,SCHED_OTHER,0);
      }
      std::cout << "FIFO/RR priority 1 applied and read back on both workers\n";
    }
    pinocchio::Model model; solver.getModel(model);
    pinocchio::Data data(model);
    Eigen::VectorXd q=Eigen::VectorXd::Zero(model.nv),out(model.nv),lo,hi;
    pinocchio::forwardKinematics(model,data,PIN_IK::toPinocchioConfiguration(model,q));
    const auto pose=pinocchio::updateFramePlacement(model,data,model.getFrameId("tip",pinocchio::BODY));
    for (int call=0;call<1000;++call) {
      require(solver.CartToJnt(q,pose,out)>0,"Configured solve failed");
      require(out.isApprox(q,1e-6),"Configured solve changed result");
    }
    solver.getLimits(lo,hi); solver.setLimits(lo,hi);
    require(solver.CartToJnt(q,pose,out)>0,"Solve after limits update failed");
    require(solver.getWorkerThreadSettings(workers[0]).cpus==std::vector<int>{first.cpus.front()},"setLimits lost affinity");
    require(solver.getWorkerThreadSettings(workers[1]).cpus==std::vector<int>{second.cpus.back()},"setLimits lost NLopt affinity");
    for (unsigned i=0;i<2;++i)
      require(ids[i]==PIN_IK::WorkerTestAccess::thread(solver,i).get_id(),"Worker recreated");
    solver.setWorkerThreadAffinity(workers[0],first.cpus);
    solver.setWorkerThreadAffinity(workers[1],second.cpus);
    solver.setWorkerThreadScheduling(workers[0],first.policy,first.priority);
    solver.setWorkerThreadScheduling(workers[1],second.policy,second.priority);
    require(equal(first,solver.getWorkerThreadSettings(workers[0])) && equal(second,solver.getWorkerThreadSettings(workers[1])),"Restore failed");
    require(pthread_getaffinity_np(pthread_self(),sizeof(caller_after),&caller_after)==0 && CPU_EQUAL(&caller_before,&caller_after),"Caller affinity changed");
    int policy; sched_param parameters{};
    require(pthread_getschedparam(pthread_self(),&policy,&parameters)==0 && policy==caller_policy && parameters.sched_priority==caller_parameters.sched_priority,"Caller scheduling changed");
    std::cout << "Worker settings, native readback, invalid requests, 1000 solves, limits, IDs and restore passed\n";
#else
    unsigned unsupported=0;
    for (auto worker : workers) {
      try { solver.setWorkerThreadAffinity(worker,{0}); }
      catch (const std::system_error& e) { require(e.code().value()==ENOTSUP,"Wrong platform error"); ++unsupported; }
      try { solver.setWorkerThreadScheduling(worker,0,0); }
      catch (const std::system_error& e) { require(e.code().value()==ENOTSUP,"Wrong platform error"); ++unsupported; }
      try { solver.getWorkerThreadSettings(worker); }
      catch (const std::system_error& e) { require(e.code().value()==ENOTSUP,"Wrong platform error"); ++unsupported; }
    }
    require(unsupported==6,"Unsupported configuration accepted");
    std::cout << "All unsupported operations reported ENOTSUP\n";
#endif
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
