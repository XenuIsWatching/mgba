/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "sio_netlink.h"

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

#define NETLINK_PROTOCOL "gba-sio-1"

/* Rendezvous roughly every 61 microseconds of emulated time. Small enough to
 * stay well inside the shortest multiplayer transfer this carries (5755
 * cycles, two units at 115200 baud), large enough that two cores are not
 * stopping to talk every few instructions. */
#define NETLINK_GRAIN 1024

/* Bus message. Packed by hand rather than shipped as a struct: both ends are
 * the same build today, but protocol_id exists so that a GameCube core could
 * speak this later, and by then a shared struct layout would be an assumption
 * nobody remembers having made. */
enum {
	NL_MODE = 1,
	NL_XFER_START,
	NL_XFER_DATA,
	NL_LINES
};
#define NL_MSG_SIZE 12

static void _write32(uint8_t* p, uint32_t v) {
	p[0] = v & 0xFF;
	p[1] = (v >> 8) & 0xFF;
	p[2] = (v >> 16) & 0xFF;
	p[3] = (v >> 24) & 0xFF;
}

static uint32_t _read32(const uint8_t* p) {
	return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static bool GBASIONetlinkInit(struct GBASIODriver* driver);
static void GBASIONetlinkDeinit(struct GBASIODriver* driver);
static void GBASIONetlinkReset(struct GBASIODriver* driver);
static void GBASIONetlinkSetMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static bool GBASIONetlinkHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode);
static int GBASIONetlinkConnectedDevices(struct GBASIODriver* driver);
static int GBASIONetlinkDeviceId(struct GBASIODriver* driver);
static uint16_t GBASIONetlinkWriteSIOCNT(struct GBASIODriver* driver, uint16_t value);
static uint16_t GBASIONetlinkWriteRCNT(struct GBASIODriver* driver, uint16_t value);
static bool GBASIONetlinkStart(struct GBASIODriver* driver);
static void GBASIONetlinkFinishMultiplayer(struct GBASIODriver* driver, uint16_t data[4]);
static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);

static uint64_t _now(struct GBASIONetlink* nl) {
	return mTimingGlobalTime(&nl->d.p->p->timing);
}

static void _refreshPeers(struct GBASIONetlink* nl) {
	unsigned count = 0;
	int id = nl->link->peers(nl->port, &count);
	if (id < 0) {
		nl->selfId = 0;
		nl->peers = 0;
		return;
	}
	nl->selfId = id;
	nl->peers = count;

	/* The bus is protocol-agnostic and will happily join five machines; a GBA
	 * multiplayer link carries four. Refuse the extra rather than indexing off
	 * the end of multiData or asking GBASIOTransferCycles for a device count it
	 * rejects, both of which turn a mis-cabled room into corrupt transfers. */
	if (nl->peers > MAX_GBAS || nl->selfId >= MAX_GBAS) {
		mLOG(GBA_SIO, WARN, "Link has %u machines on it; a GBA link cable carries %i", nl->peers, MAX_GBAS);
		nl->peers = 0;
		nl->selfId = 0;
	}
}

static void _send(struct GBASIONetlink* nl, uint64_t tick, uint8_t type, uint32_t a, uint32_t b) {
	uint8_t msg[NL_MSG_SIZE];
	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = (uint8_t) nl->selfId;
	_write32(&msg[4], a);
	_write32(&msg[8], b);
	nl->link->send(nl->port, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

/* The earliest tick this core may make something happen at. Deferring an
 * originated event to the horizon, rather than firing it the instant the guest
 * asked, is what makes the promise published to the bus true. */
static uint64_t _commitTick(struct GBASIONetlink* nl) {
	return _now(nl) + nl->horizon;
}

static void _scheduleFinish(struct GBASIONetlink* nl, uint64_t finishTick, uint32_t cyclesLate) {
	struct GBASIO* sio = nl->d.p;
	uint64_t now = _now(nl);
	int32_t delay = (finishTick > now) ? (int32_t) (finishTick - now) : 1;

	delay -= (int32_t) cyclesLate;
	if (delay < 1) {
		delay = 1;
	}

	nl->transferActive = true;
	nl->finishTick = finishTick;
	sio->siocnt |= 0x80; /* busy */
	mTimingDeschedule(&sio->p->timing, &sio->completeEvent);
	mTimingSchedule(&sio->p->timing, &sio->completeEvent, delay);
}

static void _handleMessage(struct GBASIONetlink* nl, const uint8_t* msg, unsigned from, uint32_t cyclesLate) {
	uint64_t finish;

	switch (msg[0]) {
	case NL_MODE:
		/* Nothing to enforce. A peer sitting in another mode simply never
		 * produces data, and the transfer fills with 0xFFFF exactly the way an
		 * unanswered cable does. */
		break;
	case NL_XFER_START:
		if (nl->selfId == 0) {
			/* Only the clock owner starts transfers, so this is not ours. */
			break;
		}
		finish = (uint64_t) _read32(&msg[4]);
		finish |= (uint64_t) _read32(&msg[8]) << 32;

		nl->received = 0;
		memset(nl->multiData, 0xFF, sizeof(nl->multiData));
		nl->multiData[nl->selfId] = nl->d.p->p->memory.io[GBA_REG(SIOMLT_SEND)];
		nl->received |= 1u << nl->selfId;

		_send(nl, _commitTick(nl), NL_XFER_DATA, nl->multiData[nl->selfId], 0);
		_scheduleFinish(nl, finish, cyclesLate);
		break;
	case NL_LINES:
		if (from < MAX_GBAS) {
			nl->peerLines[from] = (uint8_t) _read32(&msg[4]);
			nl->peerLinesSeen |= 1u << from;
		}
		break;
	case NL_XFER_DATA:
		if (from < MAX_GBAS) {
			nl->multiData[from] = (uint16_t) _read32(&msg[4]);
			nl->received |= 1u << from;
		}
		break;
	default:
		break;
	}
}

static void _pump(struct GBASIONetlink* nl, uint32_t cyclesLate) {
	uint8_t msg[NL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(msg);

	while (nl->link->recv(nl->port, &tick, &from, msg, &len)) {
		if (len == NL_MSG_SIZE) {
			_handleMessage(nl, msg, from, cyclesLate);
		}
		len = sizeof(msg);
	}
}

/* This machine's four data lines as the cable leaves them.
 *
 * SO of one unit goes to SI of the next, so SI is read from the peer that
 * drives SO. SC and SD are common lines shared down the whole cable, so they
 * read low if ANY unit pulls them low and high otherwise, which is what an open
 * collector bus does. A line nobody drives floats high, exactly as it does with
 * no cable in the socket, so an unlinked machine is unaffected.
 */
static uint16_t _wireLines(struct GBASIONetlink* nl, uint16_t value) {
	unsigned i;
	bool sc = true;
	bool sd = true;
	bool si = true;

	/* This machine's own contribution to the shared lines. */
	if (value & (1 << 4)) {         /* SC driven */
		sc = sc && !!(value & (1 << 0));
	}
	if (value & (1 << 5)) {         /* SD driven */
		sd = sd && !!(value & (1 << 1));
	}

	for (i = 0; i < MAX_GBAS; ++i) {
		uint8_t peer;
		if (!(nl->peerLinesSeen & (1u << i)) || (int) i == nl->selfId) {
			continue;
		}
		peer = nl->peerLines[i];
		if (peer & (1 << 4)) {
			sc = sc && !!(peer & (1 << 0));
		}
		if (peer & (1 << 5)) {
			sd = sd && !!(peer & (1 << 1));
		}
		/* SI comes from whoever is driving SO into it. With two machines that is
		 * simply the other one. */
		if (peer & (1 << 7)) {
			si = si && !!(peer & (1 << 3));
		}
	}

	/* Only the lines this machine is READING are replaced; the ones it drives
	 * read back what it wrote, as they do on hardware. */
	if (!(value & (1 << 4))) {
		value = (value & ~(uint16_t) (1 << 0)) | (sc ? (1 << 0) : 0);
	}
	if (!(value & (1 << 5))) {
		value = (value & ~(uint16_t) (1 << 1)) | (sd ? (1 << 1) : 0);
	}
	if (!(value & (1 << 6))) {
		value = (value & ~(uint16_t) (1 << 2)) | (si ? (1 << 2) : 0);
	}
	return value;
}

void GBASIONetlinkCreate(struct GBASIONetlink* nl, const struct retro_link_interface* link, unsigned port) {
	memset(nl, 0, sizeof(*nl));
	nl->d.init = GBASIONetlinkInit;
	nl->d.deinit = GBASIONetlinkDeinit;
	nl->d.reset = GBASIONetlinkReset;
	nl->d.setMode = GBASIONetlinkSetMode;
	nl->d.handlesMode = GBASIONetlinkHandlesMode;
	nl->d.connectedDevices = GBASIONetlinkConnectedDevices;
	nl->d.deviceId = GBASIONetlinkDeviceId;
	nl->d.writeSIOCNT = GBASIONetlinkWriteSIOCNT;
	nl->d.writeRCNT = GBASIONetlinkWriteRCNT;
	nl->d.start = GBASIONetlinkStart;
	nl->d.finishMultiplayer = GBASIONetlinkFinishMultiplayer;

	nl->event.context = nl;
	nl->event.name = "GBA SIO Netlink";
	nl->event.callback = _netlinkEvent;
	nl->event.priority = 0x80;

	nl->link = link;
	nl->port = port;
	nl->horizon = NETLINK_GRAIN;
	nl->grain = NETLINK_GRAIN;
	nl->mode = (enum GBASIOMode) -1;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));
}

void GBASIONetlinkDestroy(struct GBASIONetlink* nl) {
	if (nl->attached && nl->link) {
		nl->link->detach(nl->port);
		nl->attached = false;
	}
}

static bool GBASIONetlinkInit(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	int id;

	if (!nl->link) {
		return false;
	}

	id = nl->link->attach(nl->port, NETLINK_PROTOCOL, GBA_ARM7TDMI_FREQUENCY);
	if (id < 0) {
		return false;
	}
	nl->attached = true;
	nl->selfId = id;
	_refreshPeers(nl);

	mTimingSchedule(&driver->p->p->timing, &nl->event, (int32_t) nl->grain);
	return true;
}

static void GBASIONetlinkDeinit(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	mTimingDeschedule(&driver->p->p->timing, &nl->event);
	GBASIONetlinkDestroy(nl);
}

static void GBASIONetlinkReset(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	nl->transferActive = false;
	nl->received = 0;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));

	/* Put the pump back on the schedule.
	 *
	 * GBAReset calls mTimingClear before it gets here, so every event booked in
	 * init is already gone. Miss this and the link looks perfect from outside:
	 * the port is attached, the bus reports both machines, and not one byte ever
	 * crosses, because nothing is left to call advance or drain the inbox.
	 * GBASIODolphinReset does the same thing for the same reason. */
	mTimingDeschedule(&driver->p->p->timing, &nl->event);
	mTimingSchedule(&driver->p->p->timing, &nl->event, (int32_t) nl->grain);
}

static void GBASIONetlinkSetMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	nl->mode = mode;
	if (nl->attached) {
		_send(nl, _commitTick(nl), NL_MODE, (uint32_t) mode, 0);
	}
}

static bool GBASIONetlinkHandlesMode(struct GBASIODriver* driver, enum GBASIOMode mode) {
	UNUSED(driver);
	UNUSED(mode);
	/* Every mode, as the in-process lockstep driver also answers.
	 *
	 * Tempting to claim only GBA_SIO_MULTI, since that is the only one this
	 * actually carries, but handlesMode is not only about carrying:
	 * GBASIOWriteSIOCNT consults the driver for deviceId and connectedDevices
	 * ONLY when it returns true. Say no for the mode a game happens to be in
	 * when it first probes the port, and it reads back a connected count of
	 * zero, decides there is no one on the other end, and never switches to
	 * multiplayer -- so the one mode this does carry is never reached.
	 *
	 * Declining to carry a transfer belongs in start() instead, where saying so
	 * costs nothing. */
	return true;
}

static int GBASIONetlinkConnectedDevices(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	_refreshPeers(nl);
	/* mGBA counts the other machines on the cable, not including this one. */
	return nl->peers > 0 ? (int) (nl->peers - 1) : 0;
}

static int GBASIONetlinkDeviceId(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	_refreshPeers(nl);
	return nl->selfId;
}

/* Present but pass-through, exactly as the lockstep driver's are. What matters
 * is that they EXIST: GBASIOWriteSIOCNT treats a driver without them as one that
 * does not handle the write, and the value the guest reads back is decided by
 * that path rather than by this one. */
static uint16_t GBASIONetlinkWriteSIOCNT(struct GBASIODriver* driver, uint16_t value) {
	UNUSED(driver);
	return value;
}

static uint16_t GBASIONetlinkWriteRCNT(struct GBASIODriver* driver, uint16_t value) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	uint8_t mine = (uint8_t) (value & 0xFF);

	if (!nl->attached) {
		return value;
	}

	/* Publish only when something actually changed. A game polling this line in
	 * a tight loop would otherwise flood the bus with identical states. */
	if (mine != nl->lines) {
		nl->lines = mine;
		_send(nl, _commitTick(nl), NL_LINES, mine, 0);
	}

	_refreshPeers(nl);
	if (nl->peers < 2) {
		return value;
	}
	return _wireLines(nl, value);
}

static bool GBASIONetlinkStart(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	uint64_t commit;
	uint64_t finish;
	int32_t cycles;

	_refreshPeers(nl);

	if (nl->mode != GBA_SIO_MULTI) {
		/* Multiplayer is the only mode carried. A normal-mode transfer can be as
		 * short as 64 cycles, well under the commit horizon this relies on, so
		 * letting mGBA time it locally and hand the guest the 0xFFFF of an
		 * unanswered cable is more honest than carrying it badly. */
		return true;
	}

	if (nl->peers < 2) {
		/* Nothing on the other end. Let mGBA time the transfer itself and read
		 * back the 0xFFFF an unanswered cable gives. */
		return true;
	}
	if (nl->selfId != 0) {
		mLOG(GBA_SIO, DEBUG, "Secondary player attempted to start a transfer");
		return false;
	}
	if (nl->transferActive) {
		mLOG(GBA_SIO, GAME_ERROR, "Transfer restarted unexpectedly");
		return false;
	}

	commit = _commitTick(nl);
	cycles = GBASIOTransferCycles(nl->mode, nl->d.p->siocnt, (int) nl->peers - 1);
	finish = commit + (uint64_t) cycles;

	nl->received = 0;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));
	nl->multiData[0] = nl->d.p->p->memory.io[GBA_REG(SIOMLT_SEND)];
	nl->received |= 1u;

	_send(nl, commit, NL_XFER_START, (uint32_t) (finish & 0xFFFFFFFFu), (uint32_t) (finish >> 32));
	_send(nl, commit, NL_XFER_DATA, nl->multiData[0], 0);

	/* Completion is scheduled here rather than left to mGBA, because it has to
	 * land on the horizon the peers were told about and not at the instant the
	 * guest wrote the start bit. */
	_scheduleFinish(nl, finish, 0);
	return false;
}

static void GBASIONetlinkFinishMultiplayer(struct GBASIODriver* driver, uint16_t data[4]) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	unsigned i;

	nl->transferActive = false;

	if (nl->peers < 2) {
		memset(data, 0xFF, sizeof(uint16_t) * 4);
		return;
	}

	for (i = 0; i < 4; ++i) {
		bool have = i < MAX_GBAS && (nl->received & (1u << i));
		data[i] = have ? nl->multiData[i] : 0xFFFF;
	}

	if (nl->received != ((1u << nl->peers) - 1)) {
		mLOG(GBA_SIO, WARN, "MULTI transfer finished without every player's data");
	}

	nl->received = 0;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));
}

static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	struct GBASIONetlink* nl = context;
	uint64_t now = _now(nl);
	uint64_t grant;
	int32_t step = (int32_t) nl->grain;

	/* Publish before reading. A peer parked on this core's horizon cannot move
	 * until it has been told the horizon moved, and it may be sitting on the
	 * very message this core is about to want. */
	grant = nl->link->advance(nl->port, now, now + nl->horizon, now + nl->grain);
	_pump(nl, cyclesLate);

	if (grant != RETRO_LINK_UNBOUNDED && grant > now && grant - now < (uint64_t) step) {
		step = (int32_t) (grant - now);
	}

	step -= (int32_t) cyclesLate;
	if (step < 1) {
		step = 1;
	}
	mTimingSchedule(timing, &nl->event, step);
}
