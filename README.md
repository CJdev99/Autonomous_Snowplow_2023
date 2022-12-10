# Autonomous-Snowplow-2022-2023
code for 2022-2023 snowplow group.

Under Jetson_snowplow -> 3 different packages

nmea_socket_pub => GPS driver, streams parsed NMEA sentences with /navsatfix message type to the /trimble_gps/fix topic. Supported sentences: GGA, GST

navstack_pub includes our launch file

