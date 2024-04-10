/* Common functions, definitions etc. */

#pragma once

#define MEMCPY_FROM_VAR(target, source) { memcpy(target, (void*)&source, sizeof(source)); }
