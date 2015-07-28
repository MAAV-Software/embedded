# all modules that aren't executables go here
add_library(messaging SHARED
			messaging/emergency_t.c
			messaging/gains_t.c
			messaging/feedback_t.c
			messaging/raw_pose_t.c
			messaging/setpt_t.c
			messaging/lcmlite.c
			messaging/DataLinkDefines.cpp
			messaging/Encoder.cpp
			messaging/Decoder.cpp
			messaging/MessageHandler.cpp
			messaging/TransmitHandler.cpp
			messaging/DataLink.cpp
           )

# external include directories
include_directories(SYSTEM ${PROJ_INC_DIR})
