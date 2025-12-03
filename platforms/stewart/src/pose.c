#include "stewart/pose.h"

#include "stewart/geometry.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

void stewart_pose_init(struct stewart_pose *pose,
		       const struct stewart_geometry *geom)
{
	assert(pose != NULL);
	assert(geom != NULL);

	pose->rx = 0.0f;
	pose->ry = 0.0f;
	pose->rz = 0.0f;
	pose->tx = 0.0f;
	pose->ty = geom->home_height;
	pose->tz = 0.0f;
}

void stewart_pose_set(struct stewart_pose *pose, float rx, float ry, float rz,
		      float tx, float ty, float tz)
{
	assert(pose != NULL);

	pose->rx = rx;
	pose->ry = ry;
	pose->rz = rz;
	pose->tx = tx;
	pose->ty = ty;
	pose->tz = tz;
}

void stewart_pose_copy(struct stewart_pose *dest,
		       const struct stewart_pose *src)
{
	assert(dest != NULL);
	assert(src != NULL);

	memcpy(dest, src, sizeof(struct stewart_pose));
}

void stewart_pose_print(const struct stewart_pose *pose)
{
	assert(pose != NULL);

	printf("stewart_pose:\n");
	printf("  Rotation: rx=%.2f° ry=%.2f° rz=%.2f°\n", pose->rx, pose->ry,
	       pose->rz);
	printf("  Position: tx=%.2f ty=%.2f tz=%.2f mm\n", pose->tx, pose->ty,
	       pose->tz);
}