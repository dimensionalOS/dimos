# topic name on dog's side including rt/ prefix
string topic
# api_id, see https://wiki.theroboverse.com/en/unitree-go2-app-console-commands#sending-commands-to-go2
uint16 api_id
# payload for specific api_id
string parameter
# priority of the message. 0 non-priority, 1 priority
uint8 priority