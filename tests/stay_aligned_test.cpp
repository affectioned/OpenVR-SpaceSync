// Stay Aligned jump handling (issue #18 post-relocalization guard).
//
// A jump the classifier would follow as a recenter must take the held path
// when the caller flags it as a likely SLAM relocalization. The flag has to
// reach handleJump(): the follow branch moves the EKF correction before
// step() returns, so overriding ev.follow afterwards (as the first version
// of the guard did) left the alignment half-followed.

#include "StayAligned.h"

#include <cstdio>
#include <cstdlib>

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

struct RunResult
{
	int jumps = 0;
	stay::Event jumpEvent;
	uint32_t followed = 0;
	uint32_t held = 0;
};

// Head standing still (with sub-mm noise so step() doesn't drop the samples
// as duplicates), then a 30 deg / 60 cm step, then still again.
static RunResult Run(bool suspectReloc)
{
	stay::Aligner a;
	a.reset();
	RunResult r;
	uint32_t seed = 12345;
	auto noise = [&](double amp)
	{
		seed = seed * 1664525u + 1013904223u;
		return ((seed >> 8) / double(1u << 24) - 0.5) * 2.0 * amp;
	};

	const double dt = 1.0 / 90.0;
	const double jumpAt = 3.0;
	for (int i = 0; i < 90 * 5; i++)
	{
		double t = i * dt;
		bool after = t >= jumpAt;
		double yaw = (after ? stay::Rad(30.0) : 0.0) + noise(stay::Rad(0.05));
		vr::HmdVector3d_t pos = {
			(after ? 0.40 : 0.0) + noise(0.0005),
			1.70 + noise(0.0005),
			(after ? 0.45 : 0.0) + noise(0.0005),
		};
		stay::Event ev = a.step(t, quaternionFromYaw(yaw), pos, stay::Hip{}, suspectReloc);
		if (ev.jump)
		{
			r.jumps++;
			r.jumpEvent = ev;
		}
	}
	r.followed = a.recentersFollowed;
	r.held = a.jumpsHeld;
	return r;
}

int main()
{
	RunResult base = Run(false);
	CHECK(base.jumps == 1);
	CHECK(base.jumpEvent.follow);
	CHECK(!base.jumpEvent.postReloc);
	CHECK(base.followed == 1 && base.held == 0);
	std::printf("ok: baseline 30 deg / 60 cm step is followed as a recenter\n");

	RunResult reloc = Run(true);
	CHECK(reloc.jumps == 1);
	CHECK(!reloc.jumpEvent.follow);
	CHECK(reloc.jumpEvent.postReloc);
	// recentersFollowed is only bumped next to ekf.follow(), so 0 here means
	// the EKF never followed the jump.
	CHECK(reloc.followed == 0 && reloc.held == 1);
	std::printf("ok: same step with suspectReloc takes the held path, EKF not followed\n");

	if (failures)
	{
		std::printf("%d check(s) failed\n", failures);
		return 1;
	}
	std::printf("\nstay aligned tests passed\n");
	return 0;
}
