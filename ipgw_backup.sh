#!/bin/bash

# 脚本开始执行前延迟一段时间，以确保系统初始化完毕
sleep 30

# 执行 ipgw i 命令
sudo ipgw i
sudo tailscale up
exit 0
