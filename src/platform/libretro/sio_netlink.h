/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef SIO_NETLINK_H
#define SIO_NETLINK_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/internal/gba/sio.h>

#include "libretro.h"

/* A GBA link cable carried by the frontend's link bus.
 *
 * This is GBASIOLockstepCoordinator's job done one level up. The coordinator
 * is a shared structure behind a mutex, which works when several GBAs run
 * inside one program, but a libretro frontend running two cores at once
 * generally loads each from its own copy of the shared library. Two instances
 * then share no globals whatsoever and cannot find each other, so the bus has
 * to live in the frontend and the cores have to reach it through
 * RETRO_ENVIRONMENT_GET_LINK_INTERFACE.
 *
 * Only multiplayer mode is carried. The normal-mode and UART transfers are
 * short enough that the commit horizon this relies on would be longer than the
 * transfer itself, so handlesMode leaves them to mGBA's usual unlinked
 * behavior rather than pretending. */

struct GBASIONetlink {
	struct GBASIODriver d;
	struct mTimingEvent event;

	const struct retro_link_interface* link;
	unsigned port;
	bool attached;

	/* Index on the bus. Player 0 owns the clock and is the only one that may
	 * originate a transfer, matching real hardware. */
	int selfId;
	unsigned peers;

	/* How far ahead of itself this core promises not to originate anything.
	 * Must stay below the shortest transfer it will carry, or a peer could
	 * still be short of the point where it publishes its half of a transfer
	 * when the transfer is due to complete. */
	uint64_t horizon;
	uint64_t grain;

	enum GBASIOMode mode;

	/* What each machine on the cable is doing, this one included.
	 *
	 * SIOCNT bit 3 is the "all GBAs ready" flag, and GBATEK is explicit that it
	 * is hardware-driven and read-only: a game selects multiplayer mode and then
	 * READS that bit to decide whether anyone is out there. Nothing writes it, so
	 * the driver has to keep it true, which means knowing what every peer is up
	 * to. Ready means all of them are in the same mode as this one. */
	enum GBASIOMode peerModes[MAX_GBAS];
	uint32_t peerModesSeen;

	/* General-purpose line state, which is how a game finds out whether anything
	 * is on the other end at all. A GBA boots into GPIO mode, drives SO and reads
	 * SI, and decides from that; without the peer's lines it only ever reads back
	 * its own and concludes it is alone. Low byte of RCNT: bits 0-3 the SC, SD,
	 * SI and SO data, bits 4-7 their directions, 1 meaning driven. */
	uint8_t lines;
	uint8_t peerLines[MAX_GBAS];
	uint32_t peerLinesSeen;

	bool transferActive;
	uint64_t finishTick;
	uint16_t multiData[MAX_GBAS];
	uint32_t received;
};

void GBASIONetlinkCreate(struct GBASIONetlink*, const struct retro_link_interface* link, unsigned port);
void GBASIONetlinkDestroy(struct GBASIONetlink*);

CXX_GUARD_END

#endif
