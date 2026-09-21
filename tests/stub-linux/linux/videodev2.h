#pragma once
#include <cstdint>
typedef uint32_t __u32; typedef int32_t __s32;
struct v4l2_control { __u32 id; __s32 value; };
struct v4l2_ext_control { __u32 id; __u32 size; __u32 reserved2[1]; union { __s32 value; int64_t value64; void *ptr; }; };
struct v4l2_ext_controls { union { __u32 ctrl_class; __u32 which; }; __u32 count; __u32 error_idx; __u32 reserved[2]; struct v4l2_ext_control *controls; };
struct v4l2_queryctrl { __u32 id; __u32 type; char name[32]; __s32 minimum, maximum, step, default_value; __u32 flags; __u32 reserved[2]; };
#define VIDIOC_G_CTRL        0x1000u
#define VIDIOC_G_EXT_CTRLS   0x1001u
#define VIDIOC_QUERYCTRL     0x1002u
