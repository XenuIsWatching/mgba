/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef GB_SIO_NETLINK_H
#define GB_SIO_NETLINK_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/internal/gb/sio.h>

#include "libretro.h"

/* A Game Boy link cable carried by the frontend's link bus.
 *
 * The Game Boy Advance's netlink driver does not cover this. mGBA's Game Boy is
 * a separate core with a separate serial port and a separate driver interface,
 * and a GBA link cable and a Game Boy one have nothing in common but the shape
 * of the thing you push in. A cable between two Game Boys run by this core
 * reached nobody at all before this existed.
 *
 * WHAT CROSSES. A Game Boy's serial port is two machines and one wire, and
 * unlike a GBA's it has no notion of a parent: whichever unit selects the
 * internal clock (SC bit 0) drives the transfer, and that is a choice the GUEST
 * makes, both ends being identical hardware. So there are only two messages. A
 * machine says what its serial register holds and whether it is armed
 * (GBNL_STATE), and a machine that clocks a transfer says so (GBNL_CLOCK).
 *
 * The protocol id and the message layout are shared byte for byte with
 * gambatte's driver, so a Game Boy run by this core and one run by gambatte can
 * be joined by the same cable.
 *
 * WHAT DOES NOT. The four-player adapter and the Game Boy Printer are not
 * carried, and anything past a pair is refused: mGBA's own GBSIO is built for
 * MAX_GBS of two, and a bus that quietly joined three would produce transfers no
 * real cable could.
 */

enum { GBNL_PENDING_MAX = 16 };

struct GBSIONetlinkMessage {
	uint64_t tick;
	uint8_t type;
	uint8_t sb;
	uint8_t flags;
};

struct GBSIONetlink {
	struct GBSIODriver d;
	struct mTimingEvent event;

	const struct retro_link_interface* link;
	unsigned port;

	/* What the frontend gave back at attach, and what every later call takes.
	 * Not a port number: the frontend has no way to tell one core from another
	 * from a bare port, and working it out from the calling thread only holds
	 * for a core that does everything on one. */
	retro_link_port_t *handle;
	bool attached;

	int selfId;
	unsigned peers;

	/* A monotonic 64-bit clock, accumulated from mGBA's 32-bit one.
	 *
	 * No scaling: the Game Boy's timing already runs at 8388608 ticks a second
	 * whatever speed the machine is in -- a bit is 512 ticks and takes twice
	 * that in single speed -- which is exactly the rate this bus is told about.
	 *
	 * NOT mTimingGlobalTime, which looks like the right thing and is a trap:
	 * mTimingTick only advances globalCycles under ENABLE_DEBUGGERS, so in a
	 * release core it reads zero for ever. */
	int32_t lastRaw;
	bool haveRaw;
	uint64_t nowBase;

	uint64_t horizon;
	uint64_t grain;
	uint64_t grant;
	bool fastSeen;

	/* Whether this machine has rendezvoused since the last change of membership,
	 * and the tick it stops re-announcing itself at.
	 *
	 * Seating a cable rebuilds the bus, and a rebuild takes every endpoint off
	 * its timeline until it publishes again. A message to a peer in that state
	 * has nowhere to land and the bus drops it, so a driver that announces
	 * itself ONCE on a membership change announces into nothing.
	 *
	 * A DEADLINE rather than a flag cleared when the peer answers, because WHEN an
	 * answer arrives is wall-clock luck and this decides whether a message goes out
	 * at all. Under replicated netplay both peers run both machines and have to
	 * reach the same answer, so a branch taken on arrival would have one peer
	 * announce where the other stayed quiet. A tick this machine's own clock passes
	 * is the same tick on both. */
	bool anchored;
	uint64_t reannounceUntil;

	/* The peer's serial register, as of the last state it published that this
	 * machine's clock has reached. */
	uint8_t peerSb;
	bool peerArmed;

	uint8_t lastSb;
	uint8_t lastFlags;
	bool published;

	/* A transfer this machine is in the middle of, whether it clocked it or was
	 * clocked: while one runs, SB is being shifted through and holds nothing a
	 * peer should read. */
	uint64_t busyUntil;
	bool busy;

	/* A transfer a peer has clocked, waiting for this machine's own clock to
	 * reach the moment it starts. */
	bool clockReady;
	uint64_t clockAt;
	uint8_t clockSb;
	bool clockFast;

	struct GBSIONetlinkMessage pending[GBNL_PENDING_MAX];
	unsigned pendingCount;
};

void GBSIONetlinkCreate(struct GBSIONetlink*, const struct retro_link_interface* link, unsigned port);
void GBSIONetlinkDestroy(struct GBSIONetlink*);

CXX_GUARD_END

#endif
