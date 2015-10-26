PID=`ps | grep "iperf -s -w 512k -i 1" | cut -d ' ' -f 2`
if [ "$PID" != "" ]; then
    kill -9 $PID
fi
if [ "$1" = "restart" ]; then
    iperf -s -w 512k -i 1&
fi