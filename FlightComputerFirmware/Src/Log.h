// Copyright 2019 Bossa Nova Robotics, proprietary unpublished source code

#ifndef LOG_H
#define LOG_H

#include <stdio.h>

#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_RESET_COLOR   "\033[0m"

#define LOG_FORMAT(COLOR, FMT) LOG_COLOR(COLOR) "[%s] : " FMT LOG_RESET_COLOR "\r\n"

#define LOG_E(TAG, FMT, ...) printf(LOG_FORMAT(LOG_COLOR_RED, FMT), TAG, ##__VA_ARGS__)
#define LOG_W(TAG, FMT, ...) printf(LOG_FORMAT(LOG_COLOR_BROWN, FMT), TAG, ##__VA_ARGS__)
#define LOG_I(TAG, FMT, ...) printf(LOG_FORMAT(LOG_COLOR_GREEN, FMT), TAG, ##__VA_ARGS__)

#endif
