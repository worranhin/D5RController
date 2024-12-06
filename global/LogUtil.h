#pragma once
#include <iostream>

// 日志输出-----------------------------------------
#define HIGHLIGHT_(...)                                                        \
  do {                                                                         \
    printf("\033[35minfo - \033[0m" __VA_ARGS__);                              \
    printf("\n");                                                              \
  } while (false)

#define WARNING_(...)                                                          \
  do {                                                                         \
    printf("\033[33mwarn - \033[0m" __VA_ARGS__);                              \
    printf("\n");                                                              \
  } while (false)

#define PASS_(...)                                                             \
  do {                                                                         \
    printf("\033[32minfo - \033[0m" __VA_ARGS__);                              \
    printf("\n");                                                              \
  } while (false)

#define ERROR_(...)                                                            \
  do {                                                                         \
    printf("\033[31m err - \033[0m" __VA_ARGS__);                              \
    printf("\n");                                                              \
  } while (false)

#define INFO_(...)                                                             \
  do {                                                                         \
    printf("info - " __VA_ARGS__);                                             \
    printf("\n");                                                              \
  } while (false)
