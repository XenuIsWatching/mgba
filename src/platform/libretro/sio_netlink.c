/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "sio_netlink.h"

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

#define NETLINK_PROTOCOL "gba-sio-1"

/* Rendezvous roughly every 15 microseconds of emulated time.
 *
 * The ceiling is the shortest transfer carried: 5755 cycles for two units at
 * 115200 baud. The horizon is also how far the two machines may drift apart, and
 * a transfer's two halves have to meet inside it, so a quarter of the shortest
 * transfer leaves room for the reply to arrive before the master finishes.
 * At 1024 a good share of transfers completed with only one half present.
 *
 * The floor is cost: every grain is a rendezvous between two threads, so this
 * cannot go much lower without both cores spending their time synchronising
 * rather than emulating. */
#define NETLINK_GRAIN 256

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
static void _updateReady(struct GBASIONetlink* nl);
static void _send(struct GBASIONetlink* nl, uint64_t tick, uint8_t type, uint32_t a, uint32_t b);
static uint64_t _commitTick(struct GBASIONetlink* nl);

static uint64_t _now(struct GBASIONetlink* nl) {
	int32_t raw = mTimingCurrentTime(&nl->d.p->p->timing);
	int32_t delta;

	if (!nl->haveRaw) {
		nl->haveRaw = true;
		nl->lastRaw = raw;
		return nl->nowBase;
	}

	/* Signed difference, so the int32 wrapping round is just another step
	 * forward. Called at least once a grain, so a delta can never come close to
	 * needing more room than this. */
	delta = raw - nl->lastRaw;
	nl->lastRaw = raw;
	if (delta > 0) {
		nl->nowBase += (uint64_t) delta;
	}
	return nl->nowBase;
}

/* React to the cable's membership having changed.
 *
 * Split out because _refreshPeers is called from seven places and the reaction
 * used to live in only one of them, the driver's own event. Every other caller
 * updated the count silently, so whichever ran first swallowed the change and
 * the event then saw nothing to react to.
 *
 * That is not a rare race. GBASIOWriteSIOCNT asks the driver for deviceId and
 * connectedDevices on EVERY write, and a game in multiplayer writes SIOCNT
 * constantly, so the silent path nearly always won. Plugging a lead into two
 * machines that were already running left both of them reading "all GBAs ready"
 * as false for ever, and the game refuses multiplayer with a rejection noise.
 * It only appeared to work when the cable was already seated at boot, because
 * then setMode did the announcing instead. */
static void _peersChanged(struct GBASIONetlink* nl) {
	if (nl->attached) {
		/* Say again what this machine is doing. Modes are announced when they
		 * CHANGE, so a machine plugged in after the last change would otherwise
		 * never hear one, and both ends would sit waiting to be told the other
		 * was ready. */
		_send(nl, _commitTick(nl), NL_MODE, (uint32_t) nl->mode, 0);
	}
	_updateReady(nl);
}

static void _refreshPeers(struct GBASIONetlink* nl) {
	unsigned was = nl->peers;
	unsigned count = 0;
	int id = nl->link->peers(nl->port, &count);
	if (id < 0) {
		nl->selfId = 0;
		nl->peers = 0;
		if (was != 0) {
			_peersChanged(nl);
		}
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

	if (nl->peers != was) {
		_peersChanged(nl);
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

static void _handleMessage(struct GBASIONetlink* nl, const uint8_t* msg, uint64_t tick, unsigned from, uint32_t cyclesLate) {

	switch (msg[0]) {
	case NL_MODE:
		if (from < MAX_GBAS) {
			nl->peerModes[from] = (enum GBASIOMode) _read32(&msg[4]);
			nl->peerModesSeen |= 1u << from;
			_updateReady(nl);
		}
		break;
	case NL_XFER_START:
		if (nl->selfId == 0) {
			/* Only the clock owner starts transfers, so this is not ours. */
			break;
		}
		nl->received = 0;
		memset(nl->multiData, 0xFF, sizeof(nl->multiData));

		/* The word is NOT latched here. On hardware a child's SIOMLT_SEND is
		 * taken when the master clocks it, at the transfer's start; this message
		 * can arrive up to a whole horizon before that moment, and latching on
		 * arrival hands over whatever the child had written for the PREVIOUS
		 * round. That reads as a child repeating itself for ever while the
		 * master waits for it to move on. */
		nl->pendingStart = true;
		nl->pendingTick = tick;
		_scheduleFinish(nl, tick + (uint64_t) _read32(&msg[4]), cyclesLate);
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

/* Keep SIOCNT's "all GBAs ready" flag true, and RCNT's SD line with it.
 *
 * GBATEK: SIOCNT bit 3 reads "0=Bad connection, 1=All GBAs Ready" and is
 * read-only, driven by hardware. A game selects multiplayer mode and then READS
 * that bit to find out whether anyone is on the other end. Nothing ever writes
 * it, so a driver that does not maintain it leaves it reading zero for ever, and
 * the game refuses to start multiplayer -- with a rejection noise, and without
 * touching the port again, which makes it look from the outside as though it
 * never asked.
 *
 * Ready means every machine on the cable is in the same mode as this one, which
 * is the same rule the in-process lockstep driver applies.
 */
static void _updateReady(struct GBASIONetlink* nl) {
	struct GBASIO* sio = nl->d.p;
	unsigned i;
	bool ready;

	if (!sio || nl->mode != GBA_SIO_MULTI) {
		return;
	}

	/* Ready as soon as the cable has someone else on it.
	 *
	 * On hardware SD is a wire: the instant every unit is in multiplayer mode the
	 * line is high, with no delay to speak of. Here a peer's mode has to cross
	 * the bus, which costs at least a commit horizon, and the game samples this
	 * bit within a few cycles of selecting the mode -- once, at boot, and it
	 * remembers the answer. So consulting the peer's last ANNOUNCED mode reads a
	 * value that is stale by exactly the wrong amount: the peer is a microsecond
	 * behind on its way to the same mode, its previous mode says NORMAL, and the
	 * game is told the connection is bad and never asks again.
	 *
	 * Peer COUNT, unlike peer mode, is known synchronously. Two machines on one
	 * cable running the same game reach multiplayer mode within microseconds of
	 * each other, so presence is the honest answer to "is anyone there" and the
	 * staleness was an artefact of the transport rather than anything the guest
	 * should see. A peer that really is in another mode still behaves correctly:
	 * its half of a transfer fills with 0xFFFF, exactly as an unanswered cable
	 * does.
	 */
	ready = nl->peers >= 2;
	UNUSED(i);

	mLOG(GBA_SIO, DEBUG, "netlink: ready=%i (mode %i, peers %u)", (int) ready, (int) nl->mode, nl->peers);
	sio->siocnt = GBASIOMultiplayerSetReady(sio->siocnt, ready);
	sio->rcnt = GBASIORegisterRCNTSetSd(sio->rcnt, ready);
}

static void _pump(struct GBASIONetlink* nl, uint32_t cyclesLate) {
	uint8_t msg[NL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(msg);

	while (nl->link->recv(nl->port, &tick, &from, msg, &len)) {
		if (len == NL_MSG_SIZE) {
			_handleMessage(nl, msg, tick, from, cyclesLate);
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
	nl->pendingStart = false;
	nl->received = 0;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));

	/* A reset puts mGBA's cycle counter back to zero, which would read as an
	 * enormous jump backwards. Re-anchor instead, keeping the accumulated total
	 * moving forward so the bus never sees this machine's clock go back.
	 */
	nl->haveRaw = false;
	nl->linesPublished = false;

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
	mLOG(GBA_SIO, DEBUG, "netlink: mode -> %i (peers %u, id %i)", (int) mode, nl->peers, nl->selfId);
	if (nl->attached) {
		_refreshPeers(nl);
		_send(nl, _commitTick(nl), NL_MODE, (uint32_t) mode, 0);
		_updateReady(nl);
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
	 * a tight loop would otherwise flood the bus with identical states.
	 *
	 * The first write is always published, changed or not. A GBA boots by
	 * writing RCNT with every line an input, whose low byte is zero, and zero
	 * is also what this field starts at, so on the one reading either machine
	 * actually performs, "nothing changed" was true and neither ever heard
	 * from the other at all. */
	if (mine != nl->lines || !nl->linesPublished) {
		nl->lines = mine;
		nl->linesPublished = true;
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

	/* The finish time rides as the message's TICK, not in its body.
	 *
	 * Ticks are counted from each machine's own power-on, so an absolute one put
	 * in the payload means nothing at the other end: the bus converts the tick
	 * FIELD between timelines and cannot convert bytes it is only carrying. Sent
	 * as a raw number it had every transfer completing at a meaningless moment,
	 * and every one of them finished without the other half's data. So the start
	 * is stamped at the transfer's finish and the body carries only how long the
	 * transfer takes, which is a duration and means the same on both machines. */
	/* Both stamped at the commit tick, start first.
	 *
	 * The inbox is ordered by tick, so stamping the start at the FINISH tick put
	 * it behind the data: the slave stored the master's half, then processed the
	 * start, which resets the transfer and threw that half away. Every transfer
	 * then finished a player short.
	 *
	 * The duration goes in the body instead. It is a count of cycles rather than
	 * a moment, so unlike an absolute tick it means the same thing on both
	 * machines, and the slave gets its finish by adding it to the converted tick
	 * the start arrived on. */
	_send(nl, commit, NL_XFER_START, (uint32_t) cycles, 0);
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

	mLOG(GBA_SIO, DEBUG, "netlink: id %i siocnt %04X irq %i busy %i ready %i si %i xfer %04X %04X",
	     nl->selfId, driver->p->siocnt,
	     (int) !!(driver->p->siocnt & 0x4000), (int) !!(driver->p->siocnt & 0x0080),
	     (int) !!(driver->p->siocnt & 0x0008), (int) !!(driver->p->siocnt & 0x0004),
	     data[0], data[1]);

	if (nl->received != ((1u << nl->peers) - 1)) {
		mLOG(GBA_SIO, DEBUG, "netlink: id %i short: got %02X want %02X", nl->selfId, nl->received, (1u << nl->peers) - 1);
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

	/* Latch this machine's half once its clock reaches the transfer's start. */
	if (nl->pendingStart && now >= nl->pendingTick) {
		nl->pendingStart = false;
		nl->multiData[nl->selfId] = nl->d.p->p->memory.io[GBA_REG(SIOMLT_SEND)];
		nl->received |= 1u << nl->selfId;
		_send(nl, now, NL_XFER_DATA, nl->multiData[nl->selfId], 0);
	}

	/* Membership is checked here as well as on the paths the guest drives,
	 * because a cable seated while the game is not touching its serial port has
	 * to be noticed too. _refreshPeers itself announces any change. */
	_refreshPeers(nl);

	if (grant != RETRO_LINK_UNBOUNDED && grant > now && grant - now < (uint64_t) step) {
		step = (int32_t) (grant - now);
	}

	step -= (int32_t) cyclesLate;
	if (step < 1) {
		step = 1;
	}
	mTimingSchedule(timing, &nl->event, step);
}
