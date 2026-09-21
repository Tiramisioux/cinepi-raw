/* Build:
 *   c++ -std=c++17 -fsyntax-only -Wall -Wextra -I. -Itests/stub-linux \
 *       tests/driver_mode_metadata_adl_test.cpp
 *
 * This is a COMPILE test, and the only test in this suite that needs stub kernel
 * headers (tests/stub-linux/) rather than the real ones: core/driver_mode_metadata.hpp
 * reaches into <linux/videodev2.h>, which no developer Mac has, so the honest
 * alternative to stubbing was discovering the failure on the camera. That is exactly
 * what happened once: the header shipped with a helper named `xioctl`, core/options.cpp
 * has its own file-scope `xioctl` declared before the include, and because every v4l2
 * argument type lives in the global namespace, argument-dependent lookup added that one
 * beside the header's at all four call sites. Four "ambiguous overload" errors, and they
 * could only appear in a translation unit that has both -- which nothing here compiled.
 *
 * So this file is that translation unit, in miniature: it declares a file-scope xioctl
 * first, exactly as options.cpp does, then includes the header and calls into it. If the
 * header ever reintroduces an unqualified call that ADL can hijack, this stops compiling.
 */
#include <sys/ioctl.h>

static int xioctl(int fd, unsigned long ctl, void *arg);

#include "core/driver_mode_metadata.hpp"

static int xioctl(int fd, unsigned long ctl, void *arg) { return ioctl(fd, ctl, arg); }

int main()
{
	DriverModeMetadata m;
	(void)read_driver_mode_metadata(m);

	/* Call the translation unit's own one too, so both names stay in play and a
	 * future collision cannot hide behind this file only ever using one of them. */
	v4l2_control c{};
	return xioctl(-1, VIDIOC_G_CTRL, &c) == 0 ? 0 : 1;
}
