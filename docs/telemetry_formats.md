# Telemetry packet format

# Command packet format 

All of the following commands are for the drone to recieve, send by a base station on the same network. Initial implementation will work using the UDP protocol to keep the networking simple. We might change to a connection oriented protocol later.

## Base station pulse
**Data structure on port 50000:**
|name|type|unit|
|:-|:-:|:-:|
|good_pulse|bool|-|
|extra_pulse|bool|-| 

## Motor mixed packet
**Data structure on port 51000:**
|name|type|unit|
|:-|:-:|:-:|
|thrust|uint_16|-| 
|pitch|float|radians|
|roll|float|radians|
|yaw|float|radians|
|checksum|uint_8|-|

## Local $L$ velocity packet
**Data structure on port 51001:**
|name|type|unit|
|:-|:-:|:-:|
|Lvel_x|float|m/s| 
|Lvel_y|float|m/s|
|Lvel_z|float|m/s|
|checksum|uint_8|-|

## Global $G$ velocity packet
**Data structure on port 51002:**
|name|type|unit|
|:-|:-:|:-:|
|Gvel_x|float|m/s| 
|Gvel_y|float|m/s|
|Gvel_z|float|m/s|
|checksum|uint_8|-|

## Local $L$ position packet
**Data structure on port 51003:**
|name|type|unit|
|:-|:-:|:-:|
|Lpos_x|float|m| 
|Lpos_y|float|m|
|Lpos_z|float|m|
|checksum|uint_8|-|

## Global $G$ GCS position packet
**Data structure on port 51004:**
|name|type|unit|
|:-|:-:|:-:|
|Gpos_x|double|deg, longitude| 
|Gpos_y|double|deg, latitude|
|Gpos_z|double|m|
|checksum|uint_8|-|

## Global $M$ locally linearized position packet
**Data structure on port 51005:**
|name|type|unit|
|:-|:-:|:-:|
|Mpos_x|double|m, longitude| 
|Mpos_y|double|m, latitude|
|Mpos_z|double|m|
|checksum|uint_8|-|

## Set flight config packet
**Data structure on port 51006:**
|name|type|unit|
|:-|:-:|:-:|
|max_dist_step|float|m| 
|max_velocity_xy|float|m/s|
|max_velocity_z|float|m/s|
|max_pitchroll|float|radians|
|max_altitude|float|m|
|descent_bad_pulse|bool|-|
|pulse_timeout_ms|uint32_t|ms|
|checksum|uint_8|-|