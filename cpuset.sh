#! /bin/sh

## Get permission to govern CPU clock rate
sudo chmod 777 /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

## Sets CPU governor to performance
sudo echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

## Detailed documentation on the following commands can be found at:
sudo echo -1 >/proc/sys/kernel/sched_rt_runtime_us
echo 999999 > /proc/sys/kernel/sched_rt_period_us
## This sets the cpu dma latency to be as low as possible for this process
chown root:rtprio /dev/cpu_dma_latency
chmod g+rw /dev/cpu_dma_latency
echo "Real-time priority settings configured successfully"