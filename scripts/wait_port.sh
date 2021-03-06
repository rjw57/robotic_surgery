#!/bin/bash
#
# Usage:
#   wait_port.sh <host> <port>
#
# Waits up to 10 seconds for <port> to become open on <host>. Polls every half
# second.
until nc -q 0 $1 $2 < /dev/null > /dev/null; do
    echo "waiting for $1:$2..."
    sleep 0.5
done
