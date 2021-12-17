#!/bin/bash
rm -rf /tmp/pipe
mkfifo /tmp/pipe
play -t raw -r 4000 -b 8 -c 1 -e unsigned-integer /tmp/pipe
