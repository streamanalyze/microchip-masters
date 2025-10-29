#!/bin/bash
# =========================================================
# Network NAT and Forwarding Setup Script for Ubuntu
# =========================================================
# This script enables IP forwarding, configures NAT,
# adjusts rp_filter settings, and sets up nftables rules.
#
# Run with: sudo ./network_nat_setup.sh
# =========================================================

set -euxo pipefail   # Exit on errors

# ---- Configuration ----
WIRED_IFACE="enp45s0"      # Your wired interface
WIRELESS_IFACE="wlp0s20f3" # Your wireless interface
# ----------------------------------------

# Ensure script runs as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root (use sudo)."
  exit 1
fi

echo "Starting network configuration..."

# 1) Enable IPv4 forwarding
echo "Enabling IP forwarding..."
sysctl -w net.ipv4.ip_forward=1
# Make persistent
#sed -i '/^net.ipv4.ip_forward/d' /etc/sysctl.conf
#echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf

# 2) Restart wired interface
echo "Restarting wired interface: $WIRED_IFACE"
ip link set "$WIRED_IFACE" down
sleep 2
ip link set "$WIRED_IFACE" up
sleep 2

# 3) Configure iptables NAT masquerading
echo "Setting up NAT via iptables..."
iptables -t nat -A POSTROUTING -o $WIRELESS_IFACE -j MASQUERADE

# 4) Show current rp_filter values
echo "Current rp_filter settings:"
cat /proc/sys/net/ipv4/conf/all/rp_filter

# 5) Disable rp_filter on interfaces
echo "Disabling rp_filter on interfaces..."
sysctl -w net.ipv4.conf.$WIRED_IFACE.rp_filter=0
sysctl -w net.ipv4.conf.$WIRELESS_IFACE.rp_filter=0
sysctl -w net.ipv4.conf.all.rp_filter=0

# Make persistent
#{
#  echo "net.ipv4.conf.$WIRED_IFACE.rp_filter=0"
#  echo "net.ipv4.conf.$WIRELESS_IFACE.rp_filter=0"
#  echo "net.ipv4.conf.all.rp_filter=0"
#} >> /etc/sysctl.conf

# 6) Add nftables rules for forwarding
echo "Adding nftables rules..."
nft add rule ip filter forward iifname "$WIRED_IFACE" oifname "$WIRELESS_IFACE" accept
nft add rule ip filter forward iifname "$WIRELESS_IFACE" oifname "$WIRED_IFACE" ct state related,established accept

# 7) Save nftables and iptables rules
#echo "Saving nftables and iptables configuration..."
#nft list ruleset > /etc/nftables.conf
#iptables-save > /etc/iptables/rules.v4

echo "Network configuration complete!"
