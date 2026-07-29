file(READ "${NODE_SOURCE}" NODE_TEXT)

string(FIND "${NODE_TEXT}"
  "create_publisher<std_msgs::msg::UInt32>(" START_UINT32)
if(START_UINT32 EQUAL -1)
  message(FATAL_ERROR "/mission/start must be std_msgs/msg/UInt32")
endif()

string(FIND "${NODE_TEXT}"
  "create_publisher<std_msgs::msg::UInt64>(" START_CONTEXT)
if(START_CONTEXT EQUAL -1)
  message(FATAL_ERROR "/mission/start/context must be std_msgs/msg/UInt64")
endif()

foreach(REQUIRED_TEXT "\"/mission/start\"" "\"/mission/start/context\""
                      "durability_volatile()")
  string(FIND "${NODE_TEXT}" "${REQUIRED_TEXT}" REQUIRED_INDEX)
  if(REQUIRED_INDEX EQUAL -1)
    message(FATAL_ERROR "missing START event QoS/topic contract: ${REQUIRED_TEXT}")
  endif()
endforeach()

foreach(FORBIDDEN_TEXT "/fmu/in/" "RcChannels" "std_msgs::msg::Bool>(\"/mission/start\"")
  string(FIND "${NODE_TEXT}" "${FORBIDDEN_TEXT}" FORBIDDEN_INDEX)
  if(NOT FORBIDDEN_INDEX EQUAL -1)
    message(FATAL_ERROR "mission_bridge source contains forbidden writer/RC marker: ${FORBIDDEN_TEXT}")
  endif()
endforeach()
