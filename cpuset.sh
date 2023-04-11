#! /bin/sh

sudo chmod 777 /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
sudo chmod 777 /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

sudo echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
sudo echo performance > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

sudo echo -1 >/proc/sys/kernel/sched_rt_runtime_us
echo 999999 > /proc/sys/kernel/sched_rt_period_us
chown root:rtprio /dev/cpu_dma_latency
chmod g+rw /dev/cpu_dma_latency
echo "Real-time priority settings configured successfully"