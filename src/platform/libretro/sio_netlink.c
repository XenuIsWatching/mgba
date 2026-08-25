/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "sio_netlink.h"

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

#define NETLINK_PROTOCOL "gba-sio-1"

/* The GameCube lead. A different conversation entirely from the link cable, so a
 * different name: the bus refuses to join two ports that do not agree on one,
 * which is what stops a link lead being pushed into a GameCube socket. */
#define JOYLINK_PROTOCOL "gba-joy-1"

/* A JOY bus byte takes eight bit-times on the wire, and mGBA's own Dolphin
 * driver counts them at 115200 bits per second. That figure is wrong about the
 * hardware and right about compatibility: it is what both ends of this protocol
 * have always assumed, and the reply has to land where the asker expects it. */
#define JOY_BITS_PER_SECOND 115200
#define JOY_CYCLES_PER_BIT (GBA_ARM7TDMI_FREQUENCY / JOY_BITS_PER_SECOND)

/* Floor for the rendezvous grain, and what is used before a party is known.
 *
 * The floor is cost: every grain is a rendezvous between threads, so this cannot
 * go lower without the cores spending their time synchronising rather than
 * emulating. Measured on a four-handheld Quest session, the host coordinator's
 * wake path was 23.67% of all CPU cycles against 6.49% for the four emulators
 * put together, and a bench over that bus puts the cost dead linear in the
 * rendezvous RATE: at four machines, 27.5 ms of bus time at grain 256 against
 * 4.0 ms at 2048, with every other measured number unchanged.
 *
 * This is no longer the multiplayer ceiling as well; see _grainForMulti.
 *
 * The comment replaced here stated a quarter of the shortest transfer as the
 * rule and then used 256, which is a TWENTY-SECOND of a two-unit transfer, on
 * the strength of "at 1024 a good share of transfers completed with only one
 * half present". That measurement does not survive its own history. It was
 * written 2026-08-20 09:23, in the same commit as the first "get both halves of
 * a transfer" fix, and two more landed after it: the monotonic clock at 09:33,
 * and at 09:38 the one that matters -- a child latched its word when the start
 * MESSAGE arrived rather than when the master's clock reached the transfer,
 * which that commit describes as "a whole commit horizon earlier". That bug grew
 * WITH the horizon, so a larger grain made it worse, which is exactly what was
 * seen and blamed on the grain. Nobody re-measured after the fix. */
#define NETLINK_GRAIN 256

/* And how often to look when there is nothing to look FOR.
 *
 * The grain above is sized so that the two halves of a transfer meet inside the
 * horizon. That constraint only exists while a transfer can actually happen: a
 * port with nobody on the far end has nothing to be late for, and the fine
 * grain is then pure overhead paid every 256 cycles for ever.
 *
 * Measured, on Four Swords Adventures with four handhelds: each machine's SIO
 * port made 9.8 MILLION calls into the bus over one run and blocked for zero
 * milliseconds in all of them, because no link lead was in it. Across four
 * handhelds that is 39 million round trips through the coordinator's lock,
 * competing with the ports that had real work to do. The audio crackled.
 *
 * 4096 cycles is about 0.244 ms at the GBA clock, not a video frame. A cable
 * seated while the link is idle is therefore noticed at roughly 4.1 kHz, and
 * _refreshPeers returns to the active grain when anyone appears. */
#define NETLINK_IDLE_GRAIN 4096
/* Once per video frame until acknowledged. Fast enough to repair an attachment
 * race before a person can reach the menu, without flooding a partially
 * populated four-seat bus whose live indices are temporarily sparse. */
#define STATE_ANNOUNCE_RETRY (GBA_ARM7TDMI_FREQUENCY / 60)
#define STATE_TOKEN_CARTRIDGE 0x80000000u
#define STATE_TOKEN_MULTIBOOT_READY 0x40000000u
#define STATE_TOKEN_FLAGS (STATE_TOKEN_CARTRIDGE | STATE_TOKEN_MULTIBOOT_READY)


/* Multiplayer's grain, which depends on how many machines are on the wire.
 *
 * A multiplayer transfer gets LONGER as the party grows -- 5755 cycles for two
 * units, 10486 for four -- so the constraint loosens with every handheld that
 * joins, while a constant tightens it for nobody's benefit. The party is the
 * only input: the BAUD is deliberately not consulted, and the fastest one is
 * assumed instead. A game may change baud at any time, and a horizon sized to a
 * slow baud would be invalidated by that write; sizing to the fastest means the
 * real transfer is only ever longer than the one planned for.
 *
 * A quarter of the shortest, which is what the comment above _grainForMode
 * always claimed the rule was. The mechanism only needs the horizon to fit
 * inside the transfer: the master announces at commit = now + horizon, and the
 * barrier lets a peer reach that horizon "and not one tick further", so a child
 * lands exactly ON the commit tick, latches its word and sends. The master can
 * only reach the transfer's finish once the child has published far enough for
 * it to, which is commit + cycles - horizon >= commit whenever horizon <= cycles.
 * A quarter leaves three quarters of that in hand.
 *
 * Never below NETLINK_GRAIN, so a pair can never end up finer than it is today.
 */
static uint64_t _grainForMulti(struct GBASIONetlink* nl) {
	int32_t cycles;
	uint64_t grain;
	unsigned i;

	if (nl->peers < 2 || nl->peers > MAX_GBAS) {
		return NETLINK_GRAIN;
	}

	/* Only while the whole party is actually IN multiplayer.
	 *
	 * A coarse grain is a blind window: this machine looks at the wire once per
	 * grain and sees nothing in between. That is fine when everyone is settled in
	 * multiplayer and the only thing happening is transfers, and it is wrong
	 * during single-cartridge boot, where the machines flip between MULTI and
	 * NORMAL_32 repeatedly while the host hands the client a program. At a
	 * quarter of a multiplayer transfer the driver missed those turns and the
	 * upload never started: measured, the probe fell from 164258 messages and a
	 * busiest second of 11206 to 902 and 156, which is a handshake and nothing
	 * else.
	 *
	 * So the scaled grain is claimed only when every peer has announced the same
	 * mode this machine is in. Unseen counts as disagreement: a peer that has not
	 * spoken yet is not known to be anywhere.
	 *
	 * Mode agreement alone is insufficient for a three- or four-machine
	 * single-pak upload: long stretches of the BIOS protocol keep every machine
	 * in MULTI. Cartridge-less peers therefore also announce when their PC has
	 * left the BIOS for downloaded EWRAM. Keep the fine grain until every blank
	 * peer reaches that point; four-cartridge games qualify immediately. */
	for (i = 0; i < nl->peers && i < MAX_GBAS; ++i) {
		if ((int) i == nl->selfId) {
			continue;
		}
		if (!(nl->peerModesSeen & (1u << i)) || nl->peerModes[i] != nl->mode) {
			return NETLINK_GRAIN;
		}
	}

	{
		uint32_t expected = (1u << nl->peers) - 1u;
		expected &= ~(1u << nl->selfId);
		bool localCart = nl->d.p && nl->d.p->p && nl->d.p->p->memory.romSize > 0;
		bool localReady = localCart || nl->multibootReady;
		bool peersReady = (nl->peerCartridgesSeen & expected) == expected &&
		                  ((nl->peerCartridges | nl->peerMultibootReady) & expected) == expected;
		if (!localReady || !peersReady) {
			return NETLINK_GRAIN;
		}
	}

	/* Baud 3 is 115200, the fastest and so the shortest transfer. */
	cycles = GBASIOTransferCycles(GBA_SIO_MULTI, 0x0003, (int) nl->peers - 1);
	if (cycles <= 0) {
		return NETLINK_GRAIN;
	}

	grain = (uint64_t) cycles / 4;
	return grain < NETLINK_GRAIN ? NETLINK_GRAIN : grain;
}

/* Rendezvous grain for a mode, in cycles.
 *
 * Normal mode's shortest transfers are far shorter than multiplayer's: 64
 * cycles for 8-bit and 256 for 32-bit with the fast internal clock. The slow
 * clock takes 512 and 2048 cycles respectively. The grains below deliberately
 * use a quarter of the shortest case, so either clock remains safe. This is a
 * real cost paid while a game is in normal mode; single-cartridge play returns
 * to the coarse multiplayer grain after its upload completes. */
static uint64_t _grainForMode(struct GBASIONetlink* nl, enum GBASIOMode mode) {
	switch (mode) {
	case GBA_SIO_NORMAL_8:
		return 16;
	case GBA_SIO_NORMAL_32:
		return 64;
	case GBA_SIO_MULTI:
		return _grainForMulti(nl);
	default:
		return NETLINK_GRAIN;
	}
}

/* Whether this mode is one whose transfers cross the wire. */
static bool _carried(enum GBASIOMode mode) {
	return mode == GBA_SIO_MULTI || mode == GBA_SIO_NORMAL_8 || mode == GBA_SIO_NORMAL_32;
}

/* Bus message. Packed by hand rather than shipped as a struct: both ends are
 * the same build today, but protocol_id exists so that a GameCube core could
 * speak this later, and by then a shared struct layout would be an assumption
 * nobody remembers having made. */
enum {
	NL_MODE = 1,
	NL_XFER_START,
	NL_XFER_DATA,
	NL_LINES,
	NL_STATE_ACK,

	/* The GameCube lead's two. It asks, this answers; nothing else crosses. */
	NL_JOY_CMD,
	NL_JOY_REPLY
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
static uint8_t GBASIONetlinkFinishNormal8(struct GBASIODriver* driver);
static uint32_t GBASIONetlinkFinishNormal32(struct GBASIODriver* driver);
static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);
static void _updateReady(struct GBASIONetlink* nl);
static void _send(struct GBASIONetlink* nl, uint64_t tick, uint8_t type, uint32_t a, uint32_t b);
static void _sendTo(struct GBASIONetlink* nl, uint64_t tick, unsigned to, uint8_t type, uint32_t a, uint32_t b);
static uint32_t _stateToken(struct GBASIONetlink* nl);
static void _updateMultibootReady(struct GBASIONetlink* nl);
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
	/* Forget the other end. What was on the wire a moment ago is not evidence
	 * about what is on it now: a lead that has moved may be joining two different
	 * machines, and these arrays are indexed by the bus slot, which a frontend is
	 * free to hand to whoever plugs in next. Left standing, a departed machine's
	 * SC/SD/SI would go on colouring what _wireLines gives the guest out of RCNT
	 * until its replacement happened to write that register -- and a GBA writes
	 * RCNT while it boots and then leaves it alone for the rest of the session. */
	memset(nl->peerModes, 0, sizeof(nl->peerModes));
	nl->peerModesSeen = 0;
	memset(nl->peerLines, 0, sizeof(nl->peerLines));
	nl->peerLinesSeen = 0;
	nl->peerModeAcks = 0;
	nl->peerLineAcks = 0;
	nl->peerCartridges = 0;
	nl->peerCartridgesSeen = 0;
	nl->peerMultibootReady = 0;
	nl->multibootReady = false;
	nl->stateGeneration = (nl->stateGeneration + 1) & ~STATE_TOKEN_FLAGS;
	if (nl->stateGeneration == 0) {
		++nl->stateGeneration;
	}
	nl->nextStateAnnounce = 0;

	if (nl->attached) {
		/* Say everything again. Both are announced when they CHANGE, so a machine
		 * plugged in after the last change would otherwise never hear one, and
		 * both ends would sit waiting to be told the other was ready. The peers
		 * this machine just forgot are owed the same, and cannot be relied on to
		 * volunteer it: they are subject to the RCNT silence above too. */
		_send(nl, _commitTick(nl), NL_MODE, (uint32_t) nl->mode, _stateToken(nl));
		_send(nl, _commitTick(nl), NL_LINES, nl->lines, _stateToken(nl));
	}

	/* A multiplayer transfer's length depends on how many machines are on the
	 * wire, so the horizon it has to fit inside just moved. Only while nothing is
	 * in flight: a transfer announced under the old horizon has to complete under
	 * it, or the peers would be looking for it at a tick this machine no longer
	 * means. Whatever is in flight finishes first and the next one is planned at
	 * the new grain -- the event is left alone rather than re-booked, because it
	 * is at most one old grain away and re-booking it mid-transfer is the very
	 * thing being avoided. */
	if (!nl->transferActive && !nl->pendingStart) {
		nl->grain = _grainForMode(nl, nl->mode);
		nl->horizon = nl->grain;
	}

	_updateReady(nl);
}

/* The word this machine is offering, read from whichever register the current
 * mode sends from. Latched at the transfer's start rather than kept up to date,
 * because that is when the hardware takes it. */
static uint32_t _localWord(struct GBASIONetlink* nl) {
	struct GBA* gba = nl->d.p->p;
	switch (nl->mode) {
	case GBA_SIO_NORMAL_8:
		return gba->memory.io[GBA_REG(SIODATA8)] & 0xFF;
	case GBA_SIO_NORMAL_32:
		return (uint32_t) gba->memory.io[GBA_REG(SIODATA32_LO)] |
		       ((uint32_t) gba->memory.io[GBA_REG(SIODATA32_HI)] << 16);
	default:
		return gba->memory.io[GBA_REG(SIOMLT_SEND)];
	}
}

/* Where this machine's word is kept, and where a peer's arrives. */
static void _storeWord(struct GBASIONetlink* nl, unsigned index, uint32_t word) {
	if (index >= MAX_GBAS) {
		return;
	}
	if (nl->mode == GBA_SIO_MULTI) {
		nl->multiData[index] = (uint16_t) word;
	} else {
		nl->normalData[index] = word;
	}
	nl->received |= 1u << index;
}

static void _refreshPeers(struct GBASIONetlink* nl) {
	unsigned was = nl->peers;
	/* And who this machine WAS, which moves without the count moving.
	 *
	 * Seats change on their own. Two cabled pairs merging into one wider bus
	 * renumbers machines that were already playing, so a machine can go from
	 * player one to player three while the number of machines it can see never
	 * changes at all. Watching only the count misses every one of those. */
	int was_id = nl->selfId;
	unsigned count = 0;
	int id = nl->link->peers(nl->handle, &count);
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

	/* Either one. A renumber changes no count and everything else:
	 *
	 * - SIOCNT's id and slave bits are maintained in _updateReady, which only
	 *   runs from here and from setMode. Miss the renumber and the machine that
	 *   just BECAME the parent still reads back "slave", so its game never takes
	 *   the clock, while the one that became a child correctly refuses to. Then
	 *   nobody originates and the wire goes silent with both counts still
	 *   reading two, which is exactly what a seat swap did to a running game:
	 *   5110 messages before it, 5110 three watch steps later.
	 * - peerModes and peerLines are indexed BY BUS SLOT, and this function's
	 *   caller is free to hand a slot to somebody else. After a renumber they
	 *   describe the wrong machines until each of them happens to speak again.
	 *   _peersChanged forgets them and asks everyone to say it all again, which
	 *   is precisely the repair a renumber needs. */
	if (nl->peers != was || nl->selfId != was_id) {
		_peersChanged(nl);
	}
}

static void _send(struct GBASIONetlink* nl, uint64_t tick, uint8_t type, uint32_t a, uint32_t b) {
	_sendTo(nl, tick, RETRO_LINK_BROADCAST, type, a, b);
}

static uint32_t _stateToken(struct GBASIONetlink* nl) {
	uint32_t token = nl->stateGeneration & ~STATE_TOKEN_FLAGS;
	if (nl->d.p && nl->d.p->p && nl->d.p->p->memory.romSize > 0) {
		token |= STATE_TOKEN_CARTRIDGE;
	}
	if (nl->multibootReady) {
		token |= STATE_TOKEN_MULTIBOOT_READY;
	}
	return token;
}

static void _updateMultibootReady(struct GBASIONetlink* nl) {
	if (nl->multibootReady || !nl->attached || !nl->d.p || !nl->d.p->p ||
	    nl->d.p->p->memory.romSize > 0 || !nl->d.p->p->cpu) {
		return;
	}
	uint32_t pc = nl->d.p->p->cpu->gprs[ARM_PC];
	if (pc < GBA_BASE_EWRAM || pc >= GBA_BASE_ROM0) {
		return;
	}
	nl->multibootReady = true;
	nl->stateGeneration = (nl->stateGeneration + 1) & ~STATE_TOKEN_FLAGS;
	if (nl->stateGeneration == 0) {
		nl->stateGeneration = 1;
	}
	nl->peerModeAcks = 0;
	nl->peerLineAcks = 0;
	_send(nl, _commitTick(nl), NL_MODE, (uint32_t) nl->mode, _stateToken(nl));
	_send(nl, _commitTick(nl), NL_LINES, nl->lines, _stateToken(nl));
}

static void _sendTo(struct GBASIONetlink* nl, uint64_t tick, unsigned to, uint8_t type, uint32_t a, uint32_t b) {
	uint8_t msg[NL_MSG_SIZE];
	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = (uint8_t) nl->selfId;
	_write32(&msg[4], a);
	_write32(&msg[8], b);
	nl->link->send(nl->handle, tick, to, msg, sizeof(msg));
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
			uint32_t token = _read32(&msg[8]);
			nl->peerModes[from] = (enum GBASIOMode) _read32(&msg[4]);
			nl->peerModesSeen |= 1u << from;
			nl->peerCartridgesSeen |= 1u << from;
			if (token & STATE_TOKEN_CARTRIDGE) {
				nl->peerCartridges |= 1u << from;
			} else {
				nl->peerCartridges &= ~(1u << from);
			}
			if (token & STATE_TOKEN_MULTIBOOT_READY) {
				nl->peerMultibootReady |= 1u << from;
			} else {
				nl->peerMultibootReady &= ~(1u << from);
			}
			_sendTo(nl, _commitTick(nl), from, NL_STATE_ACK, 1, token);
			_updateReady(nl);

			/* The party's agreement is what licenses the coarse grain, and this
			 * is where that agreement changes. Without recomputing here, a
			 * machine keeps whatever grain it claimed when the modes last
			 * matched -- the blind window single-cartridge boot cannot survive.
			 * Only while nothing is in flight, for the same reason
			 * _peersChanged is careful: a transfer announced under one horizon
			 * has to complete under it. */
			if (!nl->transferActive && !nl->pendingStart) {
				nl->grain = _grainForMode(nl, nl->mode);
				nl->horizon = nl->grain;
			}
		}
		break;
	case NL_XFER_START:
		if (nl->mode == GBA_SIO_MULTI && nl->selfId == 0) {
			/* In multiplayer only the clock owner starts transfers, so this is
			 * not ours. Normal mode has no such rule: the clock belongs to
			 * whichever unit is driving SC, which is a choice the GUEST makes in
			 * SIOCNT and need not be the machine at the head of the cable. */
			break;
		}
		nl->received = 0;
		memset(nl->multiData, 0xFF, sizeof(nl->multiData));
		memset(nl->normalData, 0xFF, sizeof(nl->normalData));

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
			_sendTo(nl, _commitTick(nl), from, NL_STATE_ACK, 2, _read32(&msg[8]));
		}
		break;
	case NL_STATE_ACK:
		if (from < MAX_GBAS && _read32(&msg[8]) == _stateToken(nl)) {
			if (_read32(&msg[4]) & 1) {
				nl->peerModeAcks |= 1u << from;
			}
			if (_read32(&msg[4]) & 2) {
				nl->peerLineAcks |= 1u << from;
			}
		}
		break;
	case NL_XFER_DATA:
		_storeWord(nl, from, _read32(&msg[4]));
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
 * The line is shared and pulled up, so it reads high unless a unit drags it low.
 * Nothing here ever does, which is what mGBA reports with no driver installed and
 * what its own lockstep driver reports for a lone player.
 */
static void _updateReady(struct GBASIONetlink* nl) {
	struct GBASIO* sio = nl->d.p;
	bool ready;

	if (!sio || nl->mode != GBA_SIO_MULTI) {
		return;
	}

	/* High, the way an undriven open-collector line reads.
	 *
	 * SD is shared down the whole cable and pulled up: a unit drags it LOW to say
	 * it is not ready, and nothing dragging it means ready. A machine with no
	 * cable in the socket therefore reads ready, which is exactly what mGBA does
	 * with no driver installed at all (GBASIOMultiplayerFillReady on the dummy
	 * path) and what its own lockstep driver reports for a lone player.
	 *
	 * This used to be "someone else is on the cable", which is stricter than both
	 * and than the hardware, and it cost the feature its worst bug. A GBA reads
	 * this bit once, while it boots, and caches the answer; machines in a room are
	 * switched on one at a time, so the first one always read a zero no peer could
	 * afterwards undo, and refused multiplayer for the rest of its session while
	 * every count in the room said two. The only repair was to throw its game away
	 * and boot it again.
	 *
	 * Pulling the line low would mean knowing a peer is in another mode, and a
	 * peer's mode is only known as of its last announcement -- stale by a commit
	 * horizon at the exact moment the guest samples this, which reads as a bad
	 * connection when the peer is a microsecond behind on its way to the same
	 * mode. An unanswered transfer already fills with 0xFFFF, which is how a cable
	 * with nobody listening behaves on hardware too, so nothing is lost by
	 * leaving the line where hardware leaves it.
	 */
	ready = true;

	mLOG(GBA_SIO, DEBUG, "netlink: ready=%i (mode %i, peers %u, id %i)",
	     (int) ready, (int) nl->mode, nl->peers, nl->selfId);
	sio->siocnt = GBASIOMultiplayerSetReady(sio->siocnt, ready);
	sio->rcnt = GBASIORegisterRCNTSetSd(sio->rcnt, ready);

	/* And WHO this machine is on the cable, for exactly the same reason.
	 *
	 * SIOCNT's id and slave bits are hardware-driven just like the ready bit
	 * above: the cable decides them and the game only reads them. mGBA does
	 * derive them, in GBASIOWriteSIOCNT, from `id || !connected` -- but only on a
	 * WRITE, and a game sitting on a menu waiting for a partner does not write
	 * SIOCNT. So a machine that booted with nothing in its socket computed
	 * connected = 0, marked itself a SLAVE, and kept that answer after a cable
	 * arrived. A slave never originates a transfer, so with both machines holding
	 * a stale slave bit neither ever clocked the other: the party sat trading
	 * nothing while every peer count in the room correctly read two.
	 *
	 * That is what made a lead seated mid-session look like a dead cable, and it
	 * is what the room was papering over by power-cycling any machine that first
	 * saw a cable. A Game Boy Advance does not need that -- the cable is sampled
	 * continuously on hardware -- and it threw away whatever the player was doing.
	 *
	 * Same expression mGBA's own in-process lockstep driver maintains on every
	 * membership change, and the same one GBASIOWriteSIOCNT would reach on its
	 * next write. This only stops the answer going stale in between. */
	sio->siocnt = GBASIOMultiplayerSetId(sio->siocnt, nl->selfId);
	sio->siocnt = GBASIOMultiplayerSetSlave(sio->siocnt, nl->selfId || nl->peers < 2);
}

/* Answer one JOY bus command from the GameCube.
 *
 * mGBA already knows how: GBASIOJOYSendCommand takes the command byte and a
 * four-byte payload, updates JOYCNT/JOYSTAT, raises the interrupt the game is
 * waiting on, and hands back the reply to send. All that is missing is a way for
 * the two machines to reach each other, which is what the bus is.
 *
 * The reply is stamped at the moment it would actually arrive rather than at
 * once. A JOY bus exchange takes a known number of bit-times, and the GameCube
 * times its own side against that; a reply that landed instantly would be one
 * the asker is not yet listening for. */
static void _answerJoy(struct GBASIONetlink* nl, const uint8_t* msg, uint64_t tick) {
	uint8_t buffer[8];
	uint8_t reply[NL_MSG_SIZE];
	uint8_t command = msg[4];
	int bits = 8;
	int sent;

	memset(buffer, 0, sizeof(buffer));
	memcpy(buffer, &msg[5], 4);

	switch (command) {
	case JOY_RESET:
	case JOY_POLL:
		bits += 24;
		break;
	case JOY_RECV:
	case JOY_TRANS:
		bits += 40;
		break;
	default:
		break;
	}

	sent = GBASIOJOYSendCommand(&nl->d, (enum GBASIOJOYCommand) command, buffer);
	if (sent < 0) {
		sent = 0;
	}
	if (sent > 5) {
		sent = 5;
	}

	memset(reply, 0, sizeof(reply));
	reply[0] = NL_JOY_REPLY;
	reply[1] = (uint8_t) nl->selfId;
	reply[4] = (uint8_t) sent;
	memcpy(&reply[5], buffer, (size_t) sent);
	nl->link->send(nl->joyHandle, tick + (uint64_t) (bits * JOY_CYCLES_PER_BIT),
	               RETRO_LINK_BROADCAST, reply, sizeof(reply));
}


/* Drain the GameCube port. Separate from the link cable's pump because the two
 * ports carry different protocols and must never be read as one another. */
static void _pumpJoy(struct GBASIONetlink* nl) {
	uint8_t msg[NL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(msg);

	while (nl->link->recv(nl->joyHandle, &tick, &from, msg, &len)) {
		if (len == NL_MSG_SIZE && msg[0] == NL_JOY_CMD) {
			_answerJoy(nl, msg, tick);
		}
		len = sizeof(msg);
	}
}


static void _pump(struct GBASIONetlink* nl, uint32_t cyclesLate) {
	uint8_t msg[NL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(msg);

	while (nl->link->recv(nl->handle, &tick, &from, msg, &len)) {
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
	nl->d.finishNormal8 = GBASIONetlinkFinishNormal8;
	nl->d.finishNormal32 = GBASIONetlinkFinishNormal32;

	nl->event.context = nl;
	nl->event.name = "GBA SIO Netlink";
	nl->event.callback = _netlinkEvent;
	nl->event.priority = 0x80;

	nl->link = link;
	nl->port = port;
	/* The GameCube lead's port sits beside the link cable's. */
	nl->joyPort = port + 1;
	nl->horizon = NETLINK_GRAIN;
	nl->grain = NETLINK_GRAIN;
	nl->mode = (enum GBASIOMode) -1;
	memset(nl->multiData, 0xFF, sizeof(nl->multiData));
}

void GBASIONetlinkDestroy(struct GBASIONetlink* nl) {
	if (nl->attached && nl->link) {
		nl->link->detach(nl->handle);
		nl->attached = false;
	}
	if (nl->joyAttached && nl->link) {
		nl->link->detach(nl->joyHandle);
		nl->joyAttached = false;
	}
}

static bool GBASIONetlinkInit(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	if (!nl->link) {
		return false;
	}

	nl->handle = nl->link->attach(nl->port, NETLINK_PROTOCOL, GBA_ARM7TDMI_FREQUENCY);
	if (!nl->handle) {
		return false;
	}
	nl->attached = true;
	_refreshPeers(nl);

	/* The GameCube lead's port, alongside the link cable's. Attaching costs
	 * nothing while nothing is cabled to it: an unjoined port bounds no one and
	 * carries nothing, so a handheld that never meets a GameCube is unaffected. */
	nl->joyHandle = nl->link->attach(nl->joyPort, JOYLINK_PROTOCOL, GBA_ARM7TDMI_FREQUENCY);
	if (nl->joyHandle) {
		nl->joyAttached = true;
	}

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

	/* The horizon follows the mode, because the shortest transfer does. Nothing
	 * may be in flight across the change: a transfer announced under the old
	 * horizon would complete under the new one, and the peer would be looking
	 * for it at a different tick.
	 *
	 * The event is re-booked at the new grain straight away rather than at the
	 * end of the one already scheduled. A machine dropping from multiplayer's
	 * 256 into normal mode's 64 would otherwise spend its first three grains
	 * blind, which is a whole normal-mode transfer. */
	nl->transferActive = false;
	nl->pendingStart = false;
	nl->received = 0;
	nl->grain = _grainForMode(nl, mode);
	nl->horizon = nl->grain;
	if (nl->d.p && nl->d.p->p) {
		mTimingDeschedule(&nl->d.p->p->timing, &nl->event);
		mTimingSchedule(&nl->d.p->p->timing, &nl->event, (int32_t) nl->grain);
	}

	mLOG(GBA_SIO, DEBUG, "netlink: mode -> %i (peers %u, id %i)", (int) mode, nl->peers, nl->selfId);
	if (nl->attached) {
		_refreshPeers(nl);
		_send(nl, _commitTick(nl), NL_MODE, (uint32_t) mode, _stateToken(nl));
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
	if (nl->mode == GBA_SIO_JOYBUS) {
		/* A GameCube is one device on the other end, however many handhelds are
		 * on the link cable. */
		return nl->joyPeers >= 2 ? 1 : 0;
	}
	/* Membership is cached by the netlink timing event. GBASIOWriteSIOCNT calls
	 * connectedDevices and deviceId on every guest write, and multiplayer games
	 * write SIOCNT continuously. Refreshing here made each write take the
	 * frontend's global link mutex twice, in addition to the regular rendezvous.
	 *
	 * The event polls membership once per grain and is explicitly woken by a
	 * topology change, so the cached values are both timely and the same values
	 * these two callbacks would obtain back-to-back. */
	/* mGBA counts the other machines on the cable, not including this one. */
	return nl->peers > 0 ? (int) (nl->peers - 1) : 0;
}

static int GBASIONetlinkDeviceId(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	return nl->selfId;
}

/* Present but pass-through, exactly as the lockstep driver's are. What matters
 * is that they EXIST: GBASIOWriteSIOCNT treats a driver without them as one that
 * does not handle the write, and the value the guest reads back is decided by
 * that path rather than by this one. */
static uint16_t GBASIONetlinkWriteSIOCNT(struct GBASIODriver* driver, uint16_t value) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;

	/* An unlinked machine in normal mode reads SI high, which is what mGBA does
	 * with no driver installed. Worth restoring by hand because installing this
	 * driver takes that dummy path away, and with the link option now on by
	 * default that would reach every Game Boy Advance in the room rather than
	 * only the cabled ones. */
	if ((nl->mode == GBA_SIO_NORMAL_8 || nl->mode == GBA_SIO_NORMAL_32) && nl->peers < 2) {
		value = GBASIONormalFillSi(value);
	}
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
		_send(nl, _commitTick(nl), NL_LINES, mine, _stateToken(nl));
	}

	_refreshPeers(nl);

	if (nl->peers < 2) {
		/* Nothing on the other end, so nothing to wire: the guest reads back what
		 * it wrote, which is what mGBA does in this mode with no driver at all.
		 *
		 * Floating the lines high here instead was tried, on the grounds that an
		 * undriven line is pulled up and that is the more faithful reading. It
		 * makes no difference to a game picking a cable up mid-session, so it is
		 * not worth departing from the behavior every unlinked machine has had.
		 */
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

	if (!_carried(nl->mode)) {
		/* UART and JOY are not carried. Let mGBA time them and hand the guest the
		 * 0xFFFF of an unanswered cable, which is more honest than carrying them
		 * badly. */
		return true;
	}

	if (nl->peers < 2) {
		/* Nothing on the other end. Let mGBA time the transfer itself and read
		 * back the 0xFFFF an unanswered cable gives. */
		return true;
	}

	if (nl->mode == GBA_SIO_MULTI) {
		if (nl->selfId != 0) {
			mLOG(GBA_SIO, DEBUG, "Secondary player attempted to start a transfer");
			return false;
		}
	} else {
		/* Normal mode's clock owner is whichever unit is driving SC, which the
		 * GUEST chooses in SIOCNT. It is NOT the machine at the head of the
		 * cable: on a single-cartridge link the host drives the clock wherever it
		 * happens to be plugged in, and the client has no cartridge to have an
		 * opinion with.
		 *
		 * A unit on the external clock that reaches here has armed itself and is
		 * waiting to be clocked. Nothing is scheduled for it: the master's start
		 * message books its completion when it arrives, exactly as a multiplayer
		 * child's is. */
		/* Bit 0, SC, not bit 1.
		 *
		 * mGBA's names for these two do not line up with what they select.
		 * GetSc is bit 0 and is the one that says "this unit drives the clock";
		 * IsInternalSc is bit 1 and picks how FAST the internal clock runs, 256
		 * kHz or 2 MHz, which is why GBASIOTransferCycles consults it and this
		 * must not. Gating on the speed bit meant the clock owner was never
		 * recognised: the host set SC and start, this armed it as a listener
		 * instead and scheduled no completion at all, the transfer never
		 * finished, and the game spun writing SIOCNT 0x1081 sixty-seven thousand
		 * times waiting for a transfer that could not end. */
		if (!GBASIONormalGetSc(nl->d.p->siocnt)) {
			return false;
		}
		/* Any party, not just a pair.
		 *
		 * Refusing above two was wrong about what single-cartridge boot is. A
		 * host sends to up to three clients at once, and the protocol drops into
		 * normal mode partway through however many are listening; declining
		 * there let mGBA time the transfer locally and hand back the 0xFFFFFFFF
		 * of an unanswered cable, so a party of three transferred the program
		 * and then stalled with ERROR! on every screen.
		 *
		 * How the words are read is what changes with the party, and that is
		 * _peerWord's business rather than this one's. */
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
	memset(nl->normalData, 0xFF, sizeof(nl->normalData));
	_storeWord(nl, (unsigned) nl->selfId, _localWord(nl));

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
	_send(nl, commit, NL_XFER_DATA,
	      nl->mode == GBA_SIO_MULTI ? nl->multiData[nl->selfId] : nl->normalData[nl->selfId], 0);

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

/* What the other unit put on the wire, in normal mode.
 *
 * Normal mode is point to point: SO of one goes to SI of the other, both ways at
 * once, so with two machines each simply reads the other's word. mGBA's own
 * lockstep driver hands the clock owner nothing at all here, which is fine for a
 * game that only ever talks downstream and useless for single-cartridge boot,
 * where the host reads the client's answers the whole way through.
 */
static uint32_t _peerWord(struct GBASIONetlink* nl, uint32_t empty) {
	unsigned other;

	if (nl->peers < 2 || nl->mode == GBA_SIO_MULTI) {
		return empty;
	}

	if (nl->peers == 2) {
		/* A pair is crossed: each unit's SO goes to the other's SI, so each
		 * simply reads the other's word. */
		other = nl->selfId == 0 ? 1u : 0u;
	} else {
		/* More than two is a CHAIN. SO goes to the next unit's SI down the
		 * cable, so a unit reads the one before it, and the clock owner at the
		 * head has nothing feeding its SI at all -- the last unit's output goes
		 * nowhere. mGBA's own lockstep driver reads the same way. */
		if (nl->selfId == 0) {
			return empty;
		}
		other = (unsigned) nl->selfId - 1u;
	}
	if (!(nl->received & (1u << other))) {
		mLOG(GBA_SIO, DEBUG, "netlink: normal transfer finished with no answer");
		return empty;
	}
	return nl->normalData[other];
}

static uint8_t GBASIONetlinkFinishNormal8(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	uint8_t data = (uint8_t) _peerWord(nl, 0xFF);
	nl->transferActive = false;
	nl->received = 0;
	return data;
}

static uint32_t GBASIONetlinkFinishNormal32(struct GBASIODriver* driver) {
	struct GBASIONetlink* nl = (struct GBASIONetlink*) driver;
	uint32_t data = _peerWord(nl, 0xFFFFFFFF);
	nl->transferActive = false;
	nl->received = 0;
	return data;
}

static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	struct GBASIONetlink* nl = context;
	_updateMultibootReady(nl);
	uint64_t now = _now(nl);
	uint64_t grant;
	uint32_t word;
	int32_t step = (int32_t) nl->grain;

	/* Publish before reading. A peer parked on this core's horizon cannot move
	 * until it has been told the horizon moved, and it may be sitting on the
	 * very message this core is about to want. */
	/* Only when somebody is on this wire. An uncabled port's advance returns
	 * unbounded every time, and the only thing it costs is the lock it takes to
	 * say so -- millions of times over a session. */
	if (nl->peers >= 2 || nl->transferActive || nl->pendingStart) {
		grant = nl->link->advance(nl->handle, now, now + nl->horizon, now + nl->grain);
	} else {
		grant = RETRO_LINK_UNBOUNDED;
	}
	_pump(nl, cyclesLate);

	/* The GameCube lead, if one is in. Its own advance, because the bus bounds
	 * each port separately and a console on one must not be held up by a
	 * handheld on the other.
	 *
	 * This is where the socket version's clock channel went. Dolphin sends a
	 * time slice down a second socket precisely because TCP has no shared clock;
	 * the bus does, and converts between a GameCube's tick rate and a Game Boy
	 * Advance's on the way past, so there is nothing left to send. */
	if (nl->joyAttached) {
		unsigned joyCount = 0;
		if (nl->link->peers(nl->joyHandle, &joyCount) >= 0) {
			nl->joyPeers = joyCount;
		} else {
			nl->joyPeers = 0;
		}
		if (nl->joyPeers >= 2) {
			nl->link->advance(nl->joyHandle, now, now + nl->horizon, now + nl->grain);
			_pumpJoy(nl);
		}
	}

	/* Latch this machine's half once its clock reaches the transfer's start. */
	if (nl->pendingStart && now >= nl->pendingTick) {
		nl->pendingStart = false;
		word = _localWord(nl);
		_storeWord(nl, (unsigned) nl->selfId, word);
		_send(nl, now, NL_XFER_DATA, word, 0);
	}

	/* Membership is checked here as well as on the paths the guest drives,
	 * because a cable seated while the game is not touching its serial port has
	 * to be noticed too. _refreshPeers itself announces any change. */
	_refreshPeers(nl);

	/* A newly attached core is not immediately a valid message destination: it
	 * first has to publish its clock. If a membership-change announcement lands
	 * in that window the frontend drops it, so retry both pieces until every
	 * current peer has explicitly acknowledged this topology generation. */
	if (nl->attached && nl->peers >= 2 && nl->peers <= MAX_GBAS && nl->selfId < (int) nl->peers) {
		uint32_t expected = (1u << nl->peers) - 1u;
		expected &= ~(1u << nl->selfId);
		if (((nl->peerModeAcks & expected) != expected ||
		     (nl->peerLineAcks & expected) != expected) && now >= nl->nextStateAnnounce) {
			_send(nl, _commitTick(nl), NL_MODE, (uint32_t) nl->mode, _stateToken(nl));
			_send(nl, _commitTick(nl), NL_LINES, nl->lines, _stateToken(nl));
			nl->nextStateAnnounce = now + STATE_ANNOUNCE_RETRY;
		}
	}

	/* Coarsen while there is nothing on either wire and nothing in flight. The
	 * checks are cheap and local; the thing being avoided is not. */
	if (!nl->transferActive && !nl->pendingStart && nl->peers < 2 && nl->joyPeers < 2) {
		step = NETLINK_IDLE_GRAIN;
	}

	if (grant != RETRO_LINK_UNBOUNDED && grant > now && grant - now < (uint64_t) step) {
		step = (int32_t) (grant - now);
	}

	step -= (int32_t) cyclesLate;
	if (step < 1) {
		step = 1;
	}
	mTimingSchedule(timing, &nl->event, step);
}
