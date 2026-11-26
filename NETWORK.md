# Network and time sync between the Pis

How the gimbal Pi and the robot Pi are connected. All of this is set up by hand on the Pis (done 2026-11-26); nothing here is deployed from the repo. Commands on the robot Pi are run by the user.

| | Gimbal Pi | Robot Pi |
|---|---|---|
| Ethernet (`eth0`), static, no gateway | `10.42.0.1/24` | `10.42.0.2/24` |
| Wi-Fi (`wlan0`), DHCP, can change | `192.168.50.206` | `192.168.50.175` |
| Network management | NetworkManager | NetworkManager |
| chrony | time server for `10.42.0.0/24` | client of `10.42.0.1` only |
| ROS 2 (Fast DDS) | only `10.42.0.1` | only `10.42.0.2` |

SSH and internet use Wi-Fi. ROS traffic and time sync use the cable.

## 1. Wired link

Both ends have a static address with no gateway (`never-default`), so the internet route stays on Wi-Fi:

```bash
# gimbal Pi (robot Pi: 10.42.0.2/24)
sudo nmcli con add type ethernet ifname eth0 con-name gimbal-link ipv4.method manual \
  ipv4.addresses 10.42.0.1/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli con up gimbal-link
```

If a "Wired connection 1" already exists on `eth0`, it was changed with `nmcli con mod` and the same settings instead. Check: `ping 10.42.0.2` from the gimbal Pi.

## 2. Time sync (chrony)

IMU A is stamped on the gimbal Pi, `/base_command`, `/odom` and the Kobuki gyro on the robot Pi. The gimbal Pi keeps its clock from the internet (`pool` lines) and serves it to the robot Pi.

- **Gimbal Pi**, `/etc/chrony/chrony.conf` (chrony installed with `sudo apt install chrony`, which replaces `systemd-timesyncd`). Added:
  ```
  allow 10.42.0.0/24
  local stratum 10
  ```
  `local stratum 10` keeps serving time even without internet.
- **Robot Pi**, `/etc/chrony/chrony.conf`: every `pool`/`server` line commented out, and added:
  ```
  server 10.42.0.1 iburst prefer
  ```
  After a large jump, run `sudo chronyc makestep` once. Stop ROS nodes before that; they don't handle the clock jumping.
- **Check (both Pis):** `chronyc tracking` shows `Leap status: Normal` and a `System time` offset below 1 ms. On the robot Pi, `chronyc sources` marks `10.42.0.1` with `^*`.

## 3. ROS 2 only over the cable (Fast DDS)

Without this, DDS uses every interface, and ROS traffic can go over Wi-Fi (delay spikes, ARCHITECTURE.md §6).

`~/fastdds_eth0.xml` on each Pi, with that Pi's cable address (`10.42.0.1` on the gimbal Pi, `10.42.0.2` on the robot Pi):

```bash
cat > ~/fastdds_eth0.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>eth0_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.42.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="eth0_only" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>eth0_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
# robot Pi only:
sed -i 's/10.42.0.1/10.42.0.2/' ~/fastdds_eth0.xml
```

Loaded in every shell through `~/.bashrc`:

```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_eth0.xml' >> ~/.bashrc
source ~/.bashrc && ros2 daemon stop
```

- **Every ROS process needs the variable**, including `ros2 topic`, `ros2 bag` and services started by systemd, which don't read `~/.bashrc`. `src/base_command/systemd/base_command.service` sets it with an `Environment=` line.
- **After changing the file:** restart the nodes and run `ros2 daemon stop`.
- **Check (gimbal Pi):** keep `ros2 topic hz /base_command` running (about 50 Hz) and compare the bytes received per interface:
  ```bash
  a=$(cat /sys/class/net/eth0/statistics/rx_bytes); b=$(cat /sys/class/net/wlan0/statistics/rx_bytes); sleep 10; echo "eth0: $(( ($(cat /sys/class/net/eth0/statistics/rx_bytes)-a)/10 )) B/s   wlan0: $(( ($(cat /sys/class/net/wlan0/statistics/rx_bytes)-b)/10 )) B/s"
  ```
  `eth0` should carry the ROS traffic (several kB/s); `wlan0` only SSH.

## 4. Seeing topics from the laptop

With the profiles above, a laptop on Wi-Fi does not see any ROS topic. Two ways to open it up again:

**A. Foxglove bridge on the gimbal Pi (recommended, no DDS change).** The bridge is a normal ROS node on the gimbal Pi, so it receives every topic, the robot's included, over the cable. It forwards them to the laptop over a websocket on Wi-Fi.

```bash
# gimbal Pi
sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml     # port 8765
```

In the Foxglove app on the laptop, open the connection `ws://192.168.50.206:8765` (the gimbal Pi's current Wi-Fi address, see `ip -4 addr`). The ROS traffic between the Pis stays on the cable. Don't leave the bridge running during recordings: it costs CPU on the gimbal Pi.

**B. Laptop directly in DDS (`ros2 topic`, `rviz2` on the laptop).** Add the gimbal Pi's **Wi-Fi** address as a second line in its whitelist:

```xml
      <interfaceWhiteList>
        <address>10.42.0.1</address>
        <address>192.168.50.206</address>   <!-- laptop access; remove again afterwards -->
      </interfaceWhiteList>
```

Then on the gimbal Pi restart all ROS nodes and run `ros2 daemon stop`. On the laptop, use the same `ROS_DOMAIN_ID`, ROS 2 Humble, no `ROS_LOCALHOST_ONLY`, and no profile file.
- **Limitation:** the laptop then sees only the **gimbal Pi's** topics (`/gimbal/...`, `/cmd_vel`), not the robot Pi's (`/odom`, `/base_command`, ...).
- **Don't** also add the robot Pi's Wi-Fi address to its whitelist. Then both Pis would have Wi-Fi addresses in DDS, and their traffic could move back onto Wi-Fi.
- **Check afterwards:** run the byte-rate check from §3 to confirm the traffic between the Pis is still on `eth0`, and remove the extra line when you're done.

The Wi-Fi addresses come from DHCP and can change. Check them with `ip -4 addr` before using them.
