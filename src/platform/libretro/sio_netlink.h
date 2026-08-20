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
 * Multiplayer AND normal mode are carried; UART and JOY are not.
 *
 * Normal mode is what single-cartridge play runs over: the host sends a client
 * with no cartridge at all a program to run out of its RAM, in 32-bit words.
 * Those words are short -- 256 cycles against multiplayer's 5755 -- so normal
 * mode cannot share multiplayer's commit horizon, and the horizon follows the
 * mode rather than being one constant. */

struct GBASIONetlink {
	struct GBASIODriver d;
	struct mTimingEvent event;

	const struct retro_link_interface* link;
	unsigned port;
	bool attached;

	/* The GameCube cable, on a port of its own.
	 *
	 * A GBA has one EXT socket and what is in it decides who it talks to: a link
	 * lead to another handheld, or a GameCube lead to a console that treats it as
	 * a controller with a screen. The two speak nothing in common -- one is a
	 * symmetric exchange between equals, the other is a bus where the GameCube
	 * asks and the GBA answers -- so they carry different protocol ids and the
	 * bus refuses to join one to the other, which is the same refusal the room's
	 * plug groups make with the sockets.
	 *
	 * Two bus ports rather than one because the id is fixed at attach and this
	 * machine cannot know which lead someone is about to push in. The room picks
	 * by seating a cable against one port or the other. */
	unsigned joyPort;
	bool joyAttached;
	unsigned joyPeers;

	/* Index on the bus. Player 0 owns the clock and is the only one that may
	 * originate a transfer, matching real hardware. */
	int selfId;
	unsigned peers;

	/* How far ahead of itself this core promises not to originate anything.
	 * Must stay below the shortest transfer it will carry, or a peer could
	 * still be short of the point where it publishes its half of a transfer
	 * when the transfer is due to complete. */
	/* A monotonic 64-bit clock, accumulated from mGBA's 32-bit one.
	 *
	 * NOT mTimingGlobalTime, which looks like exactly the right thing and is a
	 * trap: mTimingTick only advances globalCycles under ENABLE_DEBUGGERS, so in
	 * a release core it reads zero for ever. mTimingCurrentTime is what mGBA's
	 * own SIO drivers use, and it is an int32 that wraps, so the wraps are
	 * accumulated here into something the bus can compare across machines. */
	int32_t lastRaw;
	bool haveRaw;
	uint64_t nowBase;

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
	bool linesPublished;
	uint8_t peerLines[MAX_GBAS];
	uint32_t peerLinesSeen;

	/* A transfer the master has announced but whose start tick this machine has
	 * not reached yet. Its word is latched when it gets there, not when the
	 * message arrives. */
	bool pendingStart;
	uint64_t pendingTick;

	bool transferActive;
	uint64_t finishTick;
	uint16_t multiData[MAX_GBAS];

	/* Normal mode's words, indexed the same way. Kept apart from multiData
	 * because they are 32 bits wide and because a mode change must never have
	 * one read as the other. */
	uint32_t normalData[MAX_GBAS];

	uint32_t received;
};

void GBASIONetlinkCreate(struct GBASIONetlink*, const struct retro_link_interface* link, unsigned port);
void GBASIONetlinkDestroy(struct GBASIONetlink*);

CXX_GUARD_END

#endif
