// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <cstdio>
#include <cstdlib>

#define EXIT_IF(_expression_, _message_)                                                                                       \
    if((_expression_))                                                                                                         \
    {                                                                                                                          \
        printf("%s \n - %s (File: %s, Function: %s, Line: %d)\n", _message_, #_expression_, __FILE__, __FUNCTION__, __LINE__); \
        MessageBoxA(0, _message_, NULL, MB_OK | MB_ICONHAND);                                                                  \
        exit(1);                                                                                                               \
    }

#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        MessageBoxA(0, error, NULL, MB_OK | MB_ICONHAND);                                                \
        exit(1);                                                                                         \
    }
