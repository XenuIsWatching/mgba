/* Single-cartridge multiboot under mGBA's OWN lockstep driver.
 *
 *   mgba-multiboot <rom.gba> <gba_bios.bin>
 *
 * The reference run. RetroXR drives two GBAs through a frontend bus
 * (sio_netlink.c) because two libretro instances share no globals; this drives
 * the same two GBAs through GBASIOLockstepCoordinator, which is mGBA's in-process
 * implementation of the same idea and the one the Qt frontend ships. If a
 * single-cartridge upload works here and not there, the netlink driver is at
 * fault. If it fails here too, it is not a RetroXR bug at all.
 *
 * Wired exactly as MultiplayerController::attachGame does it: a lockstep driver
 * per machine, a thread user over an mCoreThread, attach to the coordinator, then
 * setPeripheral(mPERIPH_GBA_LINK_PORT). Lockstep needs real threads -- a player
 * that must wait is put to sleep through mLockstepUser, so there is no
 * cooperative single-threaded way to run it.
 *
 * The host has the cartridge; the client has NOTHING and sits in the BIOS
 * handshake waiting to be sent a program. Success is visible rather than
 * asserted: both framebuffers are written as PPM at the end, and a client that
 * received a program is not showing the BIOS screen any more.
 */

#include <mgba/core/core.h>
#include <mgba/core/config.h>
#include <mgba/core/thread.h>
#include <mgba/core/lockstep.h>
#include <mgba/internal/gba/gba.h>
#include <mgba/gba/core.h>
#include <mgba/internal/gba/sio/lockstep.h>
#include <mgba-util/vfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIDTH 240
#define HEIGHT 160

/* mGBA's native GBA key order, not libretro's. */
#define KEY_A     (1 << 0)
#define KEY_START (1 << 3)
#define KEY_DOWN  (1 << 7)

struct Machine {
	const char* name;
	struct mCore* core;
	struct mCoreThread thread;
	struct GBASIOLockstepDriver node;
	struct mLockstepThreadUser user;
	mColor* buffer;
};

static bool machineInit(struct Machine* m, const char* name, const char* rom, const char* bios) {
	m->name = name;
	m->core = GBACoreCreate();
	if (!m->core || !m->core->init(m->core)) {
		printf("[mb] %s: core init failed\n", name);
		return false;
	}
	mCoreInitConfig(m->core, "multiboot");

	m->buffer = calloc(WIDTH * HEIGHT, sizeof(mColor));
	m->core->setVideoBuffer(m->core, m->buffer, WIDTH);

	/* The BIOS is not optional here: mGBA has no HLE for the MultiBoot SWI, so
	 * both ends need the real thing -- the host to send and the client to have
	 * anything at all to run. */
	struct VFile* biosVf = VFileOpen(bios, O_RDONLY);
	if (!biosVf) {
		printf("[mb] %s: cannot open BIOS %s\n", name, bios);
		return false;
	}
	if (!m->core->loadBIOS(m->core, biosVf, 0)) {
		printf("[mb] %s: loadBIOS refused\n", name);
		return false;
	}
	mCoreConfigSetIntValue(&m->core->config, "useBios", 1);
	mCoreConfigSetIntValue(&m->core->config, "skipBios", 0);
	mCoreLoadConfig(m->core);

	if (rom) {
		if (!mCoreLoadFile(m->core, rom)) {
			printf("[mb] %s: cannot load %s\n", name, rom);
			return false;
		}
	}
	return true;
}

static void dumpPPM(struct Machine* m, const char* path) {
	const void* pixels = NULL;
	size_t stride = 0;
	m->core->getPixels(m->core, &pixels, &stride);
	if (!pixels) {
		printf("[mb] %s: no pixels\n", m->name);
		return;
	}
	FILE* f = fopen(path, "wb");
	if (!f) {
		return;
	}
	fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
	const uint32_t* p = pixels;
	int y, x;
	for (y = 0; y < HEIGHT; ++y) {
		for (x = 0; x < WIDTH; ++x) {
			uint32_t c = p[y * stride + x];
			/* mGBA's desktop colour order is BGRA in memory on this build. */
			fputc((c >> 16) & 0xFF, f);
			fputc((c >> 8) & 0xFF, f);
			fputc(c & 0xFF, f);
		}
	}
	fclose(f);
	printf("[mb] wrote %s\n", path);
}

/* Hold a key on one machine for `press` frames, then release and wait `gap`. */
static void hold(struct Machine* m, uint32_t keys, int press, int gap) {
	uint32_t target = m->core->frameCounter(m->core) + press;
	m->core->setKeys(m->core, keys);
	while (m->core->frameCounter(m->core) < target && mCoreThreadIsActive(&m->thread)) {
		mCoreThreadContinue(&m->thread);
	}
	m->core->setKeys(m->core, 0);
	target = m->core->frameCounter(m->core) + gap;
	while (m->core->frameCounter(m->core) < target && mCoreThreadIsActive(&m->thread)) {
		mCoreThreadContinue(&m->thread);
	}
}

static void waitFrames(struct Machine* m, int n) {
	uint32_t target = m->core->frameCounter(m->core) + n;
	while (m->core->frameCounter(m->core) < target && mCoreThreadIsActive(&m->thread)) {
		mCoreThreadContinue(&m->thread);
	}
}

int main(int argc, char** argv) {
	if (argc < 3) {
		printf("usage: %s <rom.gba> <gba_bios.bin>\n", argv[0]);
		return 2;
	}
	const char* rom = argv[1];
	const char* bios = argv[2];

	struct Machine host = {0};
	struct Machine client = {0};
	if (!machineInit(&host, "host", rom, bios)) {
		return 1;
	}
	if (!machineInit(&client, "client", NULL, bios)) {
		return 1;
	}
	printf("[mb] host has %s, client has no cartridge\n", rom);

	struct GBASIOLockstepCoordinator coordinator;
	GBASIOLockstepCoordinatorInit(&coordinator);

	host.thread.core = host.core;
	client.thread.core = client.core;

	mLockstepThreadUserInit(&host.user, &host.thread);
	mLockstepThreadUserInit(&client.user, &client.thread);
	GBASIOLockstepDriverCreate(&host.node, &host.user.d);
	GBASIOLockstepDriverCreate(&client.node, &client.user.d);

	if (!mCoreThreadStart(&host.thread) || !mCoreThreadStart(&client.thread)) {
		printf("[mb] could not start core threads\n");
		return 1;
	}

	GBASIOLockstepCoordinatorAttach(&coordinator, &host.node);
	host.core->setPeripheral(host.core, mPERIPH_GBA_LINK_PORT, &host.node.d);
	GBASIOLockstepCoordinatorAttach(&coordinator, &client.node);
	client.core->setPeripheral(client.core, mPERIPH_GBA_LINK_PORT, &client.node.d);
	printf("[mb] attached %zu machines to the coordinator\n",
	       GBASIOLockstepCoordinatorAttached(&coordinator));

	/* Through the boot logo, then DOWN off Single Player onto Multiplayer and A.
	 * Same route the RetroXR probe takes, at the same counted frames. */
	waitFrames(&host, 400);
	dumpPPM(&host, "mb_host_title.ppm");
	dumpPPM(&client, "mb_client_title.ppm");

	hold(&host, KEY_START, 6, 90);
	hold(&host, KEY_DOWN, 6, 60);
	hold(&host, KEY_A, 6, 180);
	dumpPPM(&host, "mb_host_multi.ppm");

	int i;
	for (i = 0; i < 12; ++i) {
		waitFrames(&host, 120);
		printf("[mb] step%02d host frame %u, client frame %u\n", i,
		       host.core->frameCounter(host.core), client.core->frameCounter(client.core));
	}

	dumpPPM(&host, "mb_host_final.ppm");
	dumpPPM(&client, "mb_client_final.ppm");

	mCoreThreadEnd(&host.thread);
	mCoreThreadEnd(&client.thread);
	mCoreThreadJoin(&host.thread);
	mCoreThreadJoin(&client.thread);
	GBASIOLockstepCoordinatorDeinit(&coordinator);
	printf("[mb] done\n");
	return 0;
}
