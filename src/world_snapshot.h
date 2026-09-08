// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"
#include "recording_replay.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3World b3World;

// Serialize the live world into buf, interning shape geometry into rec->registry.
// On success buf holds a self-contained snapshot image, while unsupported contact storage returns -1
int b3SerializeWorld( b3World* world, b3RecBuffer* buf, b3Recording* rec );

// Restore replay state in place using geometry from the shared registry in rdr
// Returns false for an incompatible or incomplete image
bool b3DeserializeIntoShell( const uint8_t* data, int size, b3World* world, b3RecReader* rdr );
