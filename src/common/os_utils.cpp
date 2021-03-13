#include <common/os_utils.h>

#include <common/log.h>

namespace wiflx {
namespace common {

void set_thread_param (pthread_t th, const char *name, int policy, int priority, int cpu)
{
  WIFLX_LOG_FUNCTION(th << name << policy << priority << cpu);

  pthread_setname_np (th, name);

  if (-1 != cpu)
  {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(cpu, &cpu_set);
    if (pthread_setaffinity_np (th, sizeof(cpu_set_t), &cpu_set))
      WIFLX_LOG_ERROR ("failed to set cpu affinity on {}", name);
  }

  if (-1 != policy)
  {
    struct sched_param p;
    p.sched_priority = priority;
    if (pthread_setschedparam(th, policy, &p))
      WIFLX_LOG_ERROR ("failed to set policy/priority on {}", name);
  }

  {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    pthread_getaffinity_np(th, sizeof(cpu_set_t), &cpu_set);
    WIFLX_LOG_INFO("{} thread CPU mask: ", name);
    for (int i=0; i < 4; ++i)
      WIFLX_LOG_INFO("{:d} ",CPU_ISSET(i, &cpu_set));
  }

  {
    struct sched_param p;
    if (!pthread_getschedparam(th, &policy, &p))
      WIFLX_LOG_INFO ("{} thread policy {:d}, priority {:d}", name, policy, p.sched_priority);
  }
}

} // namespace common
} // namespace wiflx
