// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"
#include "vecture3d/block_grid.h"

// Fixed bound for event storage and snapshot validation.
#define V3_BLOCK_CONTACT_EVENT_CAPACITY 256

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;
typedef struct b3Contact b3Contact;

typedef struct v3BlockContactRecord
{
	v3BlockContactEvent event;
	int contactId;
	uint32_t contactGeneration;
} v3BlockContactRecord;

b3DeclareArray( v3BlockContactEvent );
b3DeclareArray( v3BlockContactRecord );

bool v3BlockContactEventsReserve( b3World* world );
void v3BlockContactEventsDestroy( b3World* world );
void v3BlockContactEventsBeginStep( b3World* world );
void v3BlockContactEventsFinishStep( b3World* world );
void v3BlockContactEventsFlushShape( b3World* world, const b3Shape* shape );
void v3BlockContactEventsPushHit( b3World* world, const b3Contact* contact, int manifoldIndex, int pointIndex,
								  const b3ContactHitEvent* nativeEvent );
