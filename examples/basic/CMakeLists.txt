cmake_minimum_required(VERSION 3.16)

# The component under test lives two directories up.
set(EXTRA_COMPONENT_DIRS "../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(asix_basic)
