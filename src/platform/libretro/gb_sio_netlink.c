/* Copyright (c) 2013-2026 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "gb_sio_netlink.h"

#include <mgba/internal/gb/gb.h>
#include <mgba/internal/gb/io.h>

/* Names the emulated wire. Byte for byte what gambatte's driver calls it, so the
 * two can be joined by one cable; a Game Boy Advance's is "gba-sio-1" and the
 * bus refuses to join the two, which is the refusal the sockets make in the
 * room. */
#define GBNL_PROTOCOL "gb-sio-1"

/* The Game Boy's timing rate. A bit is GBSIOCyclesPerTransfer ticks and takes
 * twice that outside double speed, which puts a normal transfer at 8192 ticks
 * and a byte at 8192 Hz -- so the clock runs at 8388608 whatever the machine is
 * doing. */
#define GBNL_CLOCK_RATE 8388608ull

/* Rendezvous roughly every quarter of a transfer.
 *
 * The horizon is how far two machines may drift apart, and it is also how stale
 * the peer's serial register can be when this one clocks a byte out of it. A
 * game arms its end -- writes SB, then SC -- and then waits, so the value is
 * long settled; a quarter of a byte time is comfortably inside the gap any link
 * protocol leaves between bytes.
 *
 * The floor is cost: every grain is a rendezvous between two emulation threads.
 * The fast figure is a quarter of a Game Boy Color fast transfer by the same
 * reasoning and is thirty-two times as expensive, and is paid only while a
 * machine is actually running its link at 256 kHz. */
#define GBNL_GRAIN_NORMAL 2048ull
#define GBNL_GRAIN_FAST 64ull

/* How long a machine keeps saying what it is holding after the cable moves,
 * counted in rendezvous. The window has to outlast the peer reaching its own
 * first rendezvous after the rebuild, which is one. */
#define GBNL_REANNOUNCE_GRAINS 8

enum {
	GBNL_STATE = 1,
	GBNL_CLOCK
};

enum {
	GBNL_FLAG_ARMED = 1,
	GBNL_FLAG_FAST = 2
};

#define GBNL_MSG_SIZE 8

static bool GBSIONetlinkInit(struct GBSIODriver* driver);
static void GBSIONetlinkDeinit(struct GBSIODriver* driver);
static void GBSIONetlinkWriteSB(struct GBSIODriver* driver, uint8_t value);
static uint8_t GBSIONetlinkWriteSC(struct GBSIODriver* driver, uint8_t value);
static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate);

static struct GB* _gb(struct GBSIONetlink* nl) {
	return nl->d.p->p;
}

static uint64_t _now(struct GBSIONetlink* nl) {
	int32_t raw = mTimingCurrentTime(&_gb(nl)->timing);
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

/* How long a transfer takes, in ticks. GBSIOWriteSC works this out the same way
 * and it has to agree, because this is what says when a machine stops being
 * something a peer can read. */
static uint64_t _transferTicks(struct GBSIONetlink* nl, bool fast) {
	int period = GBSIOCyclesPerTransfer[fast ? 1 : 0];
	return (uint64_t) period * (2 - _gb(nl)->doubleSpeed) * 8;
}

static void _wire(struct GBSIONetlink* nl, uint64_t tick, uint8_t type, uint8_t sb, uint8_t flags) {
	uint8_t msg[GBNL_MSG_SIZE];

	/* Nothing may be sent before this machine has rendezvoused, because the
	 * rendezvous is what anchors the origin the bus measures a tick from. A
	 * message sent between a bus rebuild and the next one carries an offset from
	 * an origin that has been thrown away, and lands in the peer's far future
	 * where its clock never reaches it. */
	if (!nl->anchored) {
		uint64_t now = _now(nl);
		nl->grant = nl->link->advance(nl->handle, now, now + nl->horizon, now);
		nl->anchored = true;
	}

	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = (uint8_t) nl->selfId;
	msg[2] = sb;
	msg[3] = flags;
	nl->link->send(nl->handle, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

/* Say what this machine's serial port is holding, if it has changed.
 *
 * Armed means the guest has set the transfer bit and is waiting to be clocked. A
 * machine in the middle of a transfer is NOT armed even though that bit is still
 * set: the peer's byte is shifted into SB as the transfer runs, so what the
 * register holds part way through is half of each byte and belongs to nobody.
 *
 * The transfer SPEED is deliberately not published. SC bit 1 belongs to whichever
 * machine clocks the transfer, and that is what GBNL_CLOCK carries.
 *
 * The registers are passed in rather than read out of memory, because GBIOWrite
 * calls the driver BEFORE it stores the value: read from memory here and this
 * machine offers the byte before the one the game just loaded, which is the kind
 * of fault a protocol survives for exactly one exchange.
 */
static void _publish(struct GBSIONetlink* nl, uint8_t sb, uint8_t sc) {
	uint8_t flags = 0;
	uint8_t out;

	if (!nl->attached) {
		return;
	}

	if (GBRegisterSCIsEnable(sc) && !nl->busy) {
		flags |= GBNL_FLAG_ARMED;
	}
	out = (flags & GBNL_FLAG_ARMED) ? sb : 0xFF;

	if (nl->published && out == nl->lastSb && flags == nl->lastFlags) {
		return;
	}
	nl->lastSb = out;
	nl->lastFlags = flags;
	nl->published = true;
	_wire(nl, _now(nl) + nl->horizon, GBNL_STATE, out, flags);
}

static void _apply(struct GBSIONetlink* nl, const struct GBSIONetlinkMessage* msg) {
	switch (msg->type) {
	case GBNL_STATE:
		nl->peerSb = msg->sb;
		nl->peerArmed = (msg->flags & GBNL_FLAG_ARMED) != 0;
		break;
	case GBNL_CLOCK:
		nl->clockReady = true;
		nl->clockAt = msg->tick;
		nl->clockSb = msg->sb;
		nl->clockFast = (msg->flags & GBNL_FLAG_FAST) != 0;
		break;
	default:
		break;
	}
}

static void _queue(struct GBSIONetlink* nl, const struct GBSIONetlinkMessage* msg) {
	if (nl->pendingCount >= GBNL_PENDING_MAX) {
		/* Nothing sane fills this: a peer publishes on change, and it cannot change
		 * more than a handful of times inside one horizon, because it can never be
		 * more than one ahead of this machine.
		 *
		 * So this drops the message and says so, rather than making room by acting on
		 * an older one early. Acting early would change what the guest reads at a
		 * moment decided by how many messages happened to have arrived, which is
		 * wall-clock luck, and that is the one thing that must not reach a guest two
		 * netplay peers are both replaying. */
		mLOG(GB_SIO, WARN, "netlink: inbox full, dropping a message the horizon"
		     " should have made impossible");
		return;
	}
	nl->pending[nl->pendingCount] = *msg;
	++nl->pendingCount;
}

/* Act on everything whose tick has come round.
 *
 * Acting on a message the moment it arrives is the whole difference between a
 * link that replays and one that does not: WHEN it arrives is a wall-clock
 * accident, when its tick comes round is not. */
static void _applyDue(struct GBSIONetlink* nl) {
	uint64_t now = _now(nl);

	while (nl->pendingCount > 0 && nl->pending[0].tick <= now) {
		_apply(nl, &nl->pending[0]);
		memmove(&nl->pending[0], &nl->pending[1],
		        sizeof(struct GBSIONetlinkMessage) * (nl->pendingCount - 1));
		--nl->pendingCount;
	}

	/* A clock nobody answered. The guest was not armed when the peer drove the
	 * line, so this machine took no part in that transfer and must not take part
	 * in it later either: leaving it standing would hand the byte to whatever the
	 * game arms NEXT, one transfer out of step. */
	if (nl->clockReady && now > nl->clockAt + _transferTicks(nl, nl->clockFast)) {
		nl->clockReady = false;
	}
}

static void _pump(struct GBSIONetlink* nl) {
	uint8_t buf[GBNL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(buf);

	if (!nl->attached) {
		return;
	}

	while (nl->link->recv(nl->handle, &tick, &from, buf, &len)) {
		if (len == GBNL_MSG_SIZE) {
			struct GBSIONetlinkMessage msg;
			msg.tick = tick;
			msg.type = buf[0];
			msg.sb = buf[2];
			msg.flags = buf[3];
			_queue(nl, &msg);
		}
		len = sizeof(buf);
	}
}

static void _refreshPeers(struct GBSIONetlink* nl) {
	unsigned was = nl->peers;
	unsigned count = 0;
	int id;

	if (!nl->attached) {
		return;
	}

	id = nl->link->peers(nl->handle, &count);
	if (id < 0) {
		nl->selfId = 0;
		nl->peers = 0;
	} else if (count > MAX_GBS) {
		/* The bus is protocol-agnostic and will happily join five machines; a
		 * Game Boy link cable joins two. */
		mLOG(GB_SIO, WARN, "Link has %u machines on it; a Game Boy link cable carries %i",
		     count, MAX_GBS);
		nl->selfId = 0;
		nl->peers = 0;
	} else {
		nl->selfId = id;
		nl->peers = count;
	}

	if (nl->peers != was) {
		/* Forget the other end, and say everything again. What was on the wire a
		 * moment ago is not evidence about what is on it now: a lead that has
		 * moved may be joining two different machines. And a machine cabled in
		 * after the last change would otherwise never hear a state at all. */
		nl->peerSb = 0xFF;
		nl->peerArmed = false;
		nl->published = false;
		nl->anchored = false;
		nl->reannounceUntil = _now(nl) + GBNL_REANNOUNCE_GRAINS * nl->grain;
		nl->clockReady = false;
		nl->pendingCount = 0;
		mLOG(GB_SIO, DEBUG, "netlink: %u machine(s) on the wire, this one is %i",
		     nl->peers, nl->selfId);
	}
}

/* Take part in a transfer a peer is clocking.
 *
 * GBSIOWriteSC arms this machine and schedules nothing, because a unit on the
 * external clock has nothing to schedule until somebody drives it. This is that
 * moment, and it books the same completion the master's own write books. */
static void _acceptClock(struct GBSIONetlink* nl) {
	struct GBSIO* sio = nl->d.p;
	struct GB* gb = _gb(nl);
	int period;

	if (!GBRegisterSCIsEnable(gb->memory.io[GB_REG_SC]) || sio->remainingBits) {
		return;
	}

	sio->pendingSB = nl->clockSb;
	sio->remainingBits = 8;
	period = GBSIOCyclesPerTransfer[nl->clockFast ? 1 : 0];
	sio->period = period;
	mTimingDeschedule(&gb->timing, &sio->event);
	mTimingSchedule(&gb->timing, &sio->event, period * (2 - gb->doubleSpeed));

	nl->clockReady = false;
	nl->fastSeen = nl->clockFast;
	nl->busy = true;
	nl->busyUntil = _now(nl) + _transferTicks(nl, nl->clockFast);
	nl->published = false;
}

void GBSIONetlinkCreate(struct GBSIONetlink* nl, const struct retro_link_interface* link, unsigned port) {
	memset(nl, 0, sizeof(*nl));
	nl->d.init = GBSIONetlinkInit;
	nl->d.deinit = GBSIONetlinkDeinit;
	nl->d.writeSB = GBSIONetlinkWriteSB;
	nl->d.writeSC = GBSIONetlinkWriteSC;

	nl->event.context = nl;
	nl->event.name = "GB SIO Netlink";
	nl->event.callback = _netlinkEvent;
	nl->event.priority = 0x31;

	nl->link = link;
	nl->port = port;
	nl->horizon = GBNL_GRAIN_NORMAL;
	nl->grain = GBNL_GRAIN_NORMAL;
	nl->peerSb = 0xFF;
}

void GBSIONetlinkDestroy(struct GBSIONetlink* nl) {
	if (nl->attached && nl->link) {
		nl->link->detach(nl->handle);
		nl->attached = false;
	}
}

static bool GBSIONetlinkInit(struct GBSIODriver* driver) {
	struct GBSIONetlink* nl = (struct GBSIONetlink*) driver;
	if (!nl->link) {
		return false;
	}

	if (!nl->attached) {
		nl->handle = nl->link->attach(nl->port, GBNL_PROTOCOL, GBNL_CLOCK_RATE);
		if (!nl->handle) {
			mLOG(GB_SIO, WARN, "netlink: the frontend refused port %u", nl->port);
			return false;
		}
		nl->attached = true;
	}

	/* A reset puts mGBA's cycle counter back to zero, which would read as an
	 * enormous jump backwards. Re-anchor instead, keeping the accumulated total
	 * moving forward so the bus never sees this machine's clock go back. */
	nl->haveRaw = false;
	nl->pendingCount = 0;
	nl->published = false;
	nl->anchored = false;
	nl->reannounceUntil = 0;
	nl->clockReady = false;
	nl->busy = false;
	nl->fastSeen = false;

	/* The peer's register goes too. What this holds was learned on a timeline
	 * that has just been thrown away -- the counter re-anchors here and anything
	 * queued for this machine is dropped -- so it is a byte from a previous life
	 * rather than evidence about the wire now.
	 *
	 * Not because the line reads 0xFF: the peer is still plugged in and still
	 * driving it, and real hardware would give whatever its shift register holds.
	 * It is that gambatte's driver clears this here, and two drivers sharing one
	 * cable must not disagree about what a reset means.
	 *
	 * Costs one transfer. The peer says its register again as soon as it is next
	 * clocked, because being clocked is itself a change it publishes. No
	 * assertion on a screen can see a single transfer, so nothing in the probe
	 * covers this -- it is a parity rule, not a measured bug. */
	nl->peerSb = 0xFF;
	nl->peerArmed = false;

	_refreshPeers(nl);

	/* Put the pump on the schedule. GBSIOReset calls GBSIOSetDriver, which comes
	 * back through here, and by then mTimingClear has taken every event booked
	 * before it. Miss this and the link looks perfect from outside: the port is
	 * attached, the bus reports both machines, and not one byte ever crosses. */
	mTimingDeschedule(&_gb(nl)->timing, &nl->event);
	mTimingSchedule(&_gb(nl)->timing, &nl->event, (int32_t) nl->grain);
	return true;
}

/* Off the schedule, but NOT off the bus.
 *
 * GBSIOReset hands the driver back to GBSIOSetDriver, which deinits and inits it
 * again, so this runs on every reset of the machine. Detaching here would take
 * the port off the wire each time a player switched a Game Boy off and on, and
 * put it back on a moment later as a stranger. Leaving the bus is
 * GBSIONetlinkDestroy's job, and retro_unload_game calls it. */
static void GBSIONetlinkDeinit(struct GBSIODriver* driver) {
	struct GBSIONetlink* nl = (struct GBSIONetlink*) driver;
	mTimingDeschedule(&_gb(nl)->timing, &nl->event);
}

static void GBSIONetlinkWriteSB(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIONetlink* nl = (struct GBSIONetlink*) driver;
	_publish(nl, value, _gb(nl)->memory.io[GB_REG_SC]);
}

static uint8_t GBSIONetlinkWriteSC(struct GBSIODriver* driver, uint8_t value) {
	struct GBSIONetlink* nl = (struct GBSIONetlink*) driver;
	struct GB* gb = _gb(nl);
	bool fast = GBRegisterSCGetClockSpeed(value);

	if (!nl->attached) {
		return value;
	}

	_pump(nl);
	_applyDue(nl);

	if (GBRegisterSCIsEnable(value) && GBRegisterSCIsShiftClock(value)) {
		/* This machine is driving. GBSIOWriteSC has already booked the
		 * completion; all that is missing is the byte coming the other way, and
		 * telling the peer that its own half starts.
		 *
		 * Announced a horizon out, not at this instant: the peer may have run as
		 * far as this machine's published horizon and no further, so a transfer
		 * stamped there is one it is guaranteed not to have passed. */
		nl->fastSeen = fast;
		if (nl->peers >= 2) {
			_wire(nl, _now(nl) + nl->horizon, GBNL_CLOCK, gb->memory.io[GB_REG_SB],
			      fast ? GBNL_FLAG_FAST : 0);
			/* What the other end had in its register. An unarmed peer reads back
			 * as the 0xFF of an open line, which is what a Game Boy clocking a
			 * cable with nothing listening on it gets. */
			nl->d.p->pendingSB = nl->peerArmed ? nl->peerSb : 0xFF;
		} else {
			nl->d.p->pendingSB = 0xFF;
		}
		nl->busy = true;
		nl->busyUntil = _now(nl) + _transferTicks(nl, fast);
		nl->published = false;
	} else if (GBRegisterSCIsEnable(value)) {
		/* Armed and waiting to be clocked. If a peer has already driven the line
		 * and this machine's clock has reached it, take that transfer now rather
		 * than at the next grain -- a game that arms and is clocked in the same
		 * breath would otherwise lose the byte to the expiry. */
		nl->published = false;
		_publish(nl, gb->memory.io[GB_REG_SB], value);
		if (nl->clockReady && _now(nl) >= nl->clockAt) {
			_acceptClock(nl);
		}
	} else {
		nl->published = false;
		_publish(nl, gb->memory.io[GB_REG_SB], value);
	}
	return value;
}

static void _netlinkEvent(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	struct GBSIONetlink* nl = context;
	struct GB* gb = _gb(nl);
	uint64_t now = _now(nl);
	int32_t step;

	nl->grain = nl->fastSeen ? GBNL_GRAIN_FAST : GBNL_GRAIN_NORMAL;
	nl->horizon = nl->grain;

	if (nl->busy && now >= nl->busyUntil) {
		nl->busy = false;
		nl->published = false;
	}

	/* Publish before reading. A peer parked on this core's horizon cannot move
	 * until it has been told the horizon moved, and it may be sitting on the very
	 * message this core is about to want. */
	nl->grant = nl->link->advance(nl->handle, now, now + nl->horizon, now + nl->grain);
	nl->anchored = true;
	_pump(nl);
	_applyDue(nl);

	_refreshPeers(nl);

	/* Say it again for a while after the cable moves. A state published while the
	 * peer was still off its timeline was dropped on the way, and this machine's
	 * serial registers may not change again for minutes -- a Game Boy waiting to
	 * be clocked writes SB and SC once and then does nothing at all. */
	if (nl->peers >= 2 && now < nl->reannounceUntil) {
		nl->published = false;
	}
	_publish(nl, gb->memory.io[GB_REG_SB], gb->memory.io[GB_REG_SC]);

	if (nl->clockReady && now >= nl->clockAt) {
		_acceptClock(nl);
	}

	step = (int32_t) nl->grain;
	if (nl->grant != RETRO_LINK_UNBOUNDED && nl->grant > now && nl->grant - now < (uint64_t) step) {
		step = (int32_t) (nl->grant - now);
	}
	step -= (int32_t) cyclesLate;
	if (step < 1) {
		step = 1;
	}
	mTimingSchedule(timing, &nl->event, step);
}
