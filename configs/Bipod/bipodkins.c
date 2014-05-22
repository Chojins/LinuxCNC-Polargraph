/*
*******************************************************************
* Description: bipodkins.c
*
* License: GPL Version 2
*    
*******************************************************************
*/


#include "motion.h"             /* these decls */
#include "rtapi_math.h"

/* ident tag */
#ifndef __GNUC__
#ifndef __attribute__
#define __attribute__(x)
#endif
#endif

#ifdef RTAPI
#include "hal.h"
struct {
    hal_float_t bx;
} *haldata = 0;

#define Bx (haldata->bx)
#else
double Bx;
#endif

#define sq(x) ((x)*(x))

int kinematicsForward(const double * joints,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
#define AD (joints[0])
#define BD (joints[1])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)

  double AD2 = sq(joints[0]);
  double BD2 = sq(joints[1]);

  double x = (AD2 - BD2 + sq(Bx)) / (2 * Bx);
  double y2 = AD2 - x * x;

  if (y2 < 0)
    return -1;

  Dx = x;
  Dy = -sqrt(y2); /*added a '-' to force the machine to always operate in -y */
  Dz = joints[2];

  if (*fflags) {
    Dy = -Dy;
  }

  return 0;

#undef AD
#undef BD
#undef Dx
#undef Dy
#undef Dz
}

int kinematicsInverse(const EmcPose * pos,
                      double * joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
#define AD (joints[0])
#define BD (joints[1])
#define Dx (pos->tran.x)
#define Dy (pos->tran.y)
#define Dz (pos->tran.z)

  double x2 = sq(Dx);
  double y2 = sq(Dy);
  AD = sqrt(x2 + y2);
  BD = sqrt(sq(Bx - Dx) + y2);
  joints[2] = Dz;

  *fflags = 0;
  if (Dy < 0.0) {
    *fflags = 1;
  }

  return 0;

#undef AD
#undef BD
#undef Dx
#undef Dy
#undef Dz
}

KINEMATICS_TYPE kinematicsType()
{
  return KINEMATICS_BOTH;
}

#ifdef MAIN

#include <stdio.h>
#include <string.h>

/*
  Interactive testing of kins.

  Syntax: a.out <Bx>
*/
int main(int argc, char *argv[])
{
#ifndef BUFFERLEN
#define BUFFERLEN 256
#endif
  char buffer[BUFFERLEN];
  char cmd[BUFFERLEN];
  EmcPose pos, vel;
  double joints[3], jointvels[3];
  char inverse;
  char flags;
  KINEMATICS_FORWARD_FLAGS fflags;

  inverse = 0;			/* forwards, by default */
  flags = 0;			/* didn't provide flags */
  fflags = 0;			/* above xy plane, by default */
  if (argc != 2 || 1 != sscanf(argv[1], "%lf", &Bx)) {
    fprintf(stderr, "syntax: %s Bx\n", argv[0]);
    return 1;
  }

  while (! feof(stdin)) {
    if (inverse) {
	printf("inv> ");
    }
    else {
	printf("fwd> ");
    }
    fflush(stdout);

    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }
    if (1 != sscanf(buffer, "%s", cmd)) {
      continue;
    }

    if (! strcmp(cmd, "quit")) {
      break;
    }
    if (! strcmp(cmd, "i")) {
      inverse = 1;
      continue;
    }
    if (! strcmp(cmd, "f")) {
      inverse = 0;
      continue;
    }
    if (! strcmp(cmd, "ff")) {
      if (1 != sscanf(buffer, "%*s %d", &fflags)) {
	printf("need forward flag\n");
      }
      continue;
    }

    if (inverse) {		/* inverse kins */
      if (3 != sscanf(buffer, "%lf %lf %lf", 
		      &pos.tran.x,
		      &pos.tran.y,
		      &pos.tran.z)) {
	printf("need X Y Z\n");
	continue;
      }
      if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	printf("inverse kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	  printf("forward kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	}
      }
    }
    else {			/* forward kins */
      if (flags) {
	if (4 != sscanf(buffer, "%lf %lf %lf %d", 
			&joints[0],
			&joints[1],
			&joints[2],
			&fflags)) {
	  printf("need 3 strut values and flag\n");
	  continue;
	}
      }
      else {
	if (3 != sscanf(buffer, "%lf %lf %lf", 
			&joints[0],
			&joints[1],
			&joints[2])) {
	  printf("need 3 strut values\n");
	  continue;
	}
      }
      if (0 != kinematicsForward(joints, &pos, &fflags, NULL)) {
	printf("forward kin error\n");
      }
      else {
	printf("%f\t%f\t%f\n", pos.tran.x, pos.tran.y, pos.tran.z);
	if (0 != kinematicsInverse(&pos, joints, NULL, &fflags)) {
	  printf("inverse kin error\n");
	}
	else {
	  printf("%f\t%f\t%f\n", joints[0], joints[1], joints[2]);
	}
      }
    }
  } /* end while (! feof(stdin)) */

  return 0;
}

#endif /* MAIN */

#ifdef RTAPI
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);

MODULE_LICENSE("GPL");



int comp_id;
int rtapi_app_main(void) {
    int res = 0;

    comp_id = hal_init("bipodkins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(*haldata));
    if(!haldata) goto error;
    Bx = 1.0;

    if((res = hal_param_float_new("bipodkins.Bx", HAL_RW, &haldata->bx, comp_id)) < 0) goto error;
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
#endif
