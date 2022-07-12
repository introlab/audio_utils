std_msgs/Header header

# Supported formats (all formats are little endian):
# - "signed_8"
# - "signed_16"
# - "signed_24"
# - "signed_padded_24"
# - "signed_32"
# - "unsigned_8"
# - "unsigned_16"
# - "unsigned_24"
# - "unsigned_padded_24"
# - "unsigned_32"
# - "float"
# - "double"
string format
uint32 channel_count
uint32 sampling_frequency
uint32 frame_sample_count

uint8[] data
