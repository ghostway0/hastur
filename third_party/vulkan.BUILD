load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "vulkan",
    hdrs = glob(["include/vulkan/*.h"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
