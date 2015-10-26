#!/bin/sh

MESSAGES="/var/log/messages"
MESSAGES0="/var/log/messages.0"

echo Content-Type: application/x-download
echo 'Content-Disposition:attachment;filename=messages"'
echo ""

if [ -e "$MESSAGES" ]; then
  echo "=============== Start message log file ==================="
  echo "`cat /var/log/messages`"
fi

if [ -e "$MESSAGES0" ]; then
  echo "=============== Start message0 log file ==================="
  echo "`cat /var/log/messages.0`"
fi
