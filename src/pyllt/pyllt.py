#
# scanCONTROL Windows Python Bindings
#
# MIT License
#
# Copyright © 2017-2018 Micro-Epsilon Messtechnik GmbH & Co. KG
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

#

import os.path
from .llt_datatypes import *

dll_name = "LLT.dll"
dll_path = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name

# Callback
buffer_cb_func = ct.CFUNCTYPE(None, ct.POINTER(ct.c_ubyte), ct.c_uint, ct.c_uint)

# DLL Loader
llt = ct.CDLL(dll_path)

# ID functions
get_device_name = llt['c_GetDeviceName']
get_device_name.restype = ct.c_int
get_device_name.argtypes = [ct.c_uint, ct.c_char_p, ct.c_uint,  ct.c_char_p, ct.c_uint]

get_llt_type = llt['c_GetLLTType']
get_llt_type.restype = ct.c_int
get_llt_type.argtypes = [ct.c_uint, ct.POINTER(ct.c_int)]

get_llt_versions = llt['c_GetLLTVersions']
get_llt_versions.restype = ct.c_int
get_llt_versions.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

# Init Functions
create_llt_device = llt['c_CreateLLTDevice']
create_llt_device.restype = ct.c_uint
create_llt_device.argtypes = [TInterfaceType]

get_device_interfaces = llt['c_GetDeviceInterfaces']
get_device_interfaces.restype = ct.c_int
get_device_interfaces.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.c_uint]

get_device_interfaces_fast = llt['c_GetDeviceInterfacesFast']
get_device_interfaces_fast.restype = ct.c_int
get_device_interfaces_fast.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.c_uint]

set_device_interface = llt['c_SetDeviceInterface']
set_device_interface.restype = ct.c_int
set_device_interface.argtypes = [ct.c_uint, ct.c_uint, ct.c_int]

get_interface_type = llt['c_GetInterfaceType']
get_interface_type.restype = ct.c_int
get_interface_type.argtypes = [ct.c_uint]

get_discovery_broadcast_target = llt['c_GetDiscoveryBroadcastTarget']
get_discovery_broadcast_target.restype = ct.c_int
get_discovery_broadcast_target.argtypes = [ct.c_uint]

set_discovery_broadcast_target = llt['c_SetDiscoveryBroadcastTarget']
set_discovery_broadcast_target.restype = ct.c_int
set_discovery_broadcast_target.argtypes = [ct.c_uint, ct.c_uint, ct.c_int]

# Delete Device
del_device = llt['c_DelDevice']
del_device.restype = ct.c_uint
del_device.argtypes = [ct.c_uint]

# Connect
connect = llt['c_Connect']
connect.restype = ct.c_int
connect.argtypes = [ct.c_uint]

disconnect = llt['c_Disconnect']
disconnect.restype = ct.c_int
disconnect.argtypes = [ct.c_uint]

# Write Config
export_llt_config = llt['c_ExportLLTConfig']
export_llt_config.restype = ct.c_int
export_llt_config.argtypes = [ct.c_uint, ct.c_char_p]

export_llt_config_string = llt['c_ExportLLTConfigString']
export_llt_config_string.restype = ct.c_int
export_llt_config_string.argtypes = [ct.c_uint, ct.c_char_p, ct.c_int]

# Read Config
import_llt_config = llt['c_ImportLLTConfig']
import_llt_config.restype = ct.c_int
import_llt_config.argtypes = [ct.c_uint, ct.c_char_p, ct.c_int]

import_llt_config_string = llt['c_ImportLLTConfigString']
import_llt_config_string.restype = ct.c_int
import_llt_config_string.argtypes = [ct.c_uint, ct.c_char_p, ct.c_int, ct.c_int]

# Transfer and Profile Functions
transfer_profiles = llt['c_TransferProfiles']
transfer_profiles.restype = ct.c_int
transfer_profiles.argtypes = [ct.c_uint, TTransferProfileType, ct.c_int]

transfer_video_stream = llt['c_TransferVideoStream']
transfer_video_stream.restype = ct.c_int
transfer_video_stream.argtypes = [ct.c_uint, TTransferVideoType, ct.c_int, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

get_actual_profile = llt['c_GetActualProfile']
get_actual_profile.restype = ct.c_int
get_actual_profile.argtypes = [ct.c_uint, ct.POINTER(ct.c_ubyte), ct.c_uint, TProfileConfig, ct.POINTER(ct.c_int)]

get_profile = llt['c_GetProfile']
get_profile.restype = ct.c_int
get_profile.argtypes = [ct.c_uint]

multi_shot = llt['c_MultiShot']
multi_shot.restype = ct.c_int
multi_shot.argtypes = [ct.c_uint, ct.c_uint]

set_hold_buffers_for_polling = llt['c_SetHoldBuffersForPolling']
set_hold_buffers_for_polling.restype = ct.c_int
set_hold_buffers_for_polling.argtypes = [ct.c_uint, ct.c_uint]

get_hold_buffers_for_polling = llt['c_GetHoldBuffersForPolling']
get_hold_buffers_for_polling.restype = ct.c_int
get_hold_buffers_for_polling.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

# Convert Profiles
convert_profile_2_values = llt['c_ConvertProfile2Values']
convert_profile_2_values.restype = ct.c_int
convert_profile_2_values.argtypes = [ct.c_uint, ct.POINTER(ct.c_ubyte), ct.c_uint, TProfileConfig, TScannerType, ct.c_uint,
                                  ct.c_uint, ct.POINTER(ct.c_ushort), ct.POINTER(ct.c_ushort), ct.POINTER(ct.c_ushort),
                                  ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_uint),
                                  ct.POINTER(ct.c_uint)]

convert_part_profile_2_values = llt['c_ConvertPartProfile2Values']
convert_part_profile_2_values.restype = ct.c_int
convert_part_profile_2_values.argtypes = [ct.c_uint, ct.POINTER(ct.c_ubyte), ct.POINTER(TPartialProfile), TScannerType,
                                      ct.c_int, ct.c_uint, ct.POINTER(ct.c_ushort), ct.POINTER(ct.c_ushort),
                                      ct.POINTER(ct.c_ushort), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                                      ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

convert_container_2_values = llt['c_ConvertContainer2Values']
convert_container_2_values.restype = ct.c_int
convert_container_2_values.argtypes = [ct.c_uint, TConvertContainerParameter]       

# Timestamp
timestamp_2_time_and_count = llt['c_Timestamp2TimeAndCount']
timestamp_2_time_and_count.restype = None
timestamp_2_time_and_count.argtypes = [ct.POINTER(ct.c_ubyte), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                                   ct.POINTER(ct.c_uint)]

timestamp_2_cmm_trigger_and_in_count = llt['c_Timestamp2CmmTriggerAndInCounter']
timestamp_2_cmm_trigger_and_in_count.restype = None
timestamp_2_cmm_trigger_and_in_count.argtypes = [ct.POINTER(ct.c_ubyte), ct.POINTER(ct.c_uint), ct.POINTER(ct.c_int),
                                             ct.POINTER(ct.c_int), ct.POINTER(ct.c_uint)]

# Set Functions
set_feature = llt['c_SetFeature']
set_feature.restype = ct.c_int
set_feature.argtypes = [ct.c_uint, ct.c_uint, ct.c_uint]

set_feature_group = llt['c_SetFeatureGroup']
set_feature_group.restype = ct.c_int
set_feature_group.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint), ct.c_uint]

set_buffer_count = llt['c_SetBufferCount']
set_buffer_count.restype = ct.c_int
set_buffer_count.argtypes = [ct.c_uint, ct.c_uint]

set_main_reflection = llt['c_SetMainReflection']
set_main_reflection.restype = ct.c_int
set_main_reflection.argtypes = [ct.c_uint, ct.c_uint]

set_max_file_size = llt['c_SetMaxFileSize']
set_max_file_size.restype = ct.c_int
set_max_file_size.argtypes = [ct.c_uint, ct.c_uint]

set_packet_size = llt['c_SetPacketSize']
set_packet_size.restype = ct.c_int
set_packet_size.argtypes = [ct.c_uint, ct.c_uint]

set_profile_config = llt['c_SetProfileConfig']
set_profile_config.restype = ct.c_int
set_profile_config.argtypes = [ct.c_uint, TProfileConfig]

set_resolution = llt['c_SetResolution']
set_resolution.restype = ct.c_int
set_resolution.argtypes = [ct.c_uint, ct.c_uint]

set_profile_container_size = llt['c_SetProfileContainerSize']
set_profile_container_size.restype = ct.c_int
set_profile_container_size.argtypes = [ct.c_uint, ct.c_uint, ct.c_uint]

set_ethernet_heartbeat_timeout = llt['c_SetEthernetHeartbeatTimeout']
set_ethernet_heartbeat_timeout.restype = ct.c_int
set_ethernet_heartbeat_timeout.argtypes = [ct.c_uint, ct.c_uint]

# Get Functions
get_feature = llt['c_GetFeature']
get_feature.restype = ct.c_int
get_feature.argtypes = [ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint)]

get_min_max_packet_size = llt['c_GetMinMaxPacketSize']
get_min_max_packet_size.restype = ct.c_int
get_min_max_packet_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

get_resolutions = llt['c_GetResolutions']
get_resolutions.restype = ct.c_int
get_resolutions.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.c_uint]

get_buffer_count = llt['c_GetBufferCount']
get_buffer_count.restype = ct.c_int
get_buffer_count.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_main_reflection = llt['c_GetMainReflection']
get_main_reflection.restype = ct.c_int
get_main_reflection.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_max_file_size = llt['c_GetMaxFileSize']
get_max_file_size.restype = ct.c_int
get_max_file_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_packet_size = llt['c_GetPacketSize']
get_packet_size.restype = ct.c_int
get_packet_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_profile_config = llt['c_GetProfileConfig']
get_profile_config.restype = ct.c_int
get_profile_config.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_resolution = llt['c_GetResolution']
get_resolution.restype = ct.c_int
get_resolution.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

get_profile_container_size = llt['c_GetProfileContainerSize']
get_profile_container_size.restype = ct.c_int
get_profile_container_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

get_max_profile_container_size = llt['c_GetMaxProfileContainerSize']
get_max_profile_container_size.restype = ct.c_int
get_max_profile_container_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

get_ethernet_heartbeat_timeout = llt['c_GetEthernetHeartbeatTimeout']
get_ethernet_heartbeat_timeout.restype = ct.c_int
get_ethernet_heartbeat_timeout.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint)]

# Maintenance parameter
get_actual_user_mode = llt['c_GetActualUserMode']
get_actual_user_mode.restype = ct.c_int
get_actual_user_mode.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

read_write_user_mode = llt['c_ReadWriteUserModes']
read_write_user_mode.restype = ct.c_int
read_write_user_mode.argtypes = [ct.c_uint, ct.c_int, ct.c_uint]

save_global_parameters = llt['c_SaveGlobalParameter']
save_global_parameters.restype = ct.c_int
save_global_parameters.argtypes = [ct.c_uint]

trigger_profile = llt['c_TriggerProfile']
trigger_profile.restype = ct.c_int
trigger_profile.argtypes = [ct.c_uint]

trigger_container = llt['c_TriggerContainer']
trigger_container.restype = ct.c_int
trigger_container.argtypes = [ct.c_uint]

trigger_container_enable = llt['c_ContainerTriggerEnable']
trigger_container_enable.restype = ct.c_int
trigger_container_enable.argtypes = [ct.c_uint]

trigger_container_disable = llt['c_ContainerTriggerDisable']
trigger_container_disable.restype = ct.c_int
trigger_container_disable.argtypes = [ct.c_uint]

flush_container = llt['c_FlushContainer']
flush_container.restype = ct.c_int
flush_container.argtypes = [ct.c_uint]

# Partial profiles function
get_partial_profile_unit_size = llt['c_GetPartialProfileUnitSize']
get_partial_profile_unit_size.restype = ct.c_int
get_partial_profile_unit_size.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

get_partial_profile = llt['c_GetPartialProfile']
get_partial_profile.restype = ct.c_int
get_partial_profile.argtypes = [ct.c_uint, ct.POINTER(TPartialProfile)]

set_partial_profile = llt['c_SetPartialProfile']
set_partial_profile.restype = ct.c_int
set_partial_profile.argtypes = [ct.c_uint, ct.POINTER(TPartialProfile)]

# Is Functions
is_interface_type = llt['c_IsInterfaceType']
is_interface_type.restype = ct.c_int
is_interface_type.argtypes = [ct.c_uint, ct.c_int]

is_transfering_profiles = llt['c_IsTransferingProfiles']
is_transfering_profiles.restype = ct.c_int
is_transfering_profiles.argtypes = [ct.c_uint]

# PostProcessing
read_post_processing_parameter = llt['c_ReadPostProcessingParameter']
read_post_processing_parameter.restype = ct.c_int
read_post_processing_parameter.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.c_uint]

write_post_processing_parameter = llt['c_WritePostProcessingParameter']
write_post_processing_parameter.restype = ct.c_int
write_post_processing_parameter.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.c_uint]

convert_profiles_2_module_results = llt['c_ConvertProfile2ModuleResult']
convert_profiles_2_module_results.restype = ct.c_int
convert_profiles_2_module_results.argtypes = [ct.c_uint, ct.POINTER(ct.c_ubyte), ct.c_uint, ct.POINTER(ct.c_ubyte), ct.c_uint,
                                        ct.POINTER(TPartialProfile)]

# Load/Save Functions
save_profiles = llt['c_SaveProfiles']
save_profiles.restype = ct.c_int
save_profiles.argtypes = [ct.c_uint, ct.c_char_p, TFileType]

load_profiles = llt['c_LoadProfiles']
load_profiles.restype = ct.c_int
load_profiles.argtypes = [ct.c_uint, ct.c_char_p, ct.POINTER(TPartialProfile), ct.POINTER(ct.c_uint),
                         ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

load_profiles_get_pos = llt['c_LoadProfilesGetPos']
load_profiles_get_pos.restype = ct.c_int
load_profiles_get_pos.argtypes = [ct.c_uint, ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint)]

load_profiles_set_pos = llt['c_LoadProfilesSetPos']
load_profiles_set_pos.restype = ct.c_int
load_profiles_set_pos.argtypes = [ct.c_uint, ct.c_uint]

# Register Functions
register_callback = llt['c_RegisterCallback']
register_callback.restype = ct.c_int
register_callback.argtypes = [ct.c_uint, TCallbackType, buffer_cb_func, ct.c_uint]

register_error_msg = llt['c_RegisterErrorMsg']
register_error_msg.restype = ct.c_int
register_error_msg.argtypes = [ct.c_uint, ct.c_uint, ct.POINTER(ct.c_int), ct.POINTER(ct.c_uint)]

# Special CMM Trigger Functions
start_transmission_and_cmm_trigger = llt['c_StartTransmissionAndCmmTrigger']
start_transmission_and_cmm_trigger.restype = ct.c_int
start_transmission_and_cmm_trigger.argtypes = [ct.c_uint, ct.c_uint, TTransferProfileType, ct.c_uint, ct.c_char_p, TFileType,
                                          ct.c_uint]

stop_transmission_and_cmm_trigger = llt['c_StopTransmissionAndCmmTrigger']
stop_transmission_and_cmm_trigger.restype = ct.c_int
stop_transmission_and_cmm_trigger.argtypes = [ct.c_uint, ct.c_int, ct.c_uint]
