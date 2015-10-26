#!/bin/sh

print_result_with_br_tag()
{
  echo "$1" | while IFS= read -r line
  do
    echo "$line<br/>"
  done
}

echo Content-Type: text/html
echo ""
echo "page generated on `date`<br/>"
echo "Vendor: Belkin<br/>"
echo "Model Name: WeMo Link<br/>"
echo "<br/>"
echo "Firmware version: `cat /etc/fw_version`<br/>"
echo "Linux: `cat /proc/version`<br/>"
echo "<br/><br/>"
echo "<a href=\"/cgi-bin/download.cgi\" download=\"messages\">Download f/w log message</a><br/><br/>"
echo "<< uptime info >><br/>"
RESULT="`uptime`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< ps info >><br/>"
RESULT="`ps`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< nvram_get info >><br/>"
RESULT="`nvram_get`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< ifconfig info >><br/>"
RESULT="`ifconfig`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< iwconfig info >><br/>"
RESULT="`iwconfig`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< Memory Use info >><br/>"
echo "free<br/>"
RESULT="`free`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
echo "<< subdevice list >><br/>"
RESULT="`subdevicetest list`"
print_result_with_br_tag "$RESULT"
echo "<br/>"
