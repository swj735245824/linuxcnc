/* userkfuncs.c: template file for a user set of
** switchable kinematics functions.
**
** Example Usage (for customizing the genser-switchkins module):
** (works with rtpreempt only rtai --> Makefile needs work)
**
**        LDIR is LinuxCNC git root directory
**        UDIR is user directory (not in LinuxCNC git tree)
**  1) $ cp LDIR/src/emc/kinematics/userkfuncs.c  UDIR/my_userk.c
**  2) $ edit   UDIR/my_userk.c as required
**  3) $ source LDIR/scripts/rip-environment
**  4) For genser-switchkins module use make command line option:
**     $ cd LDIR/src
**     $ userkfuncs=UDIR/my_userk.c make && sudo make setuid
*/

// typical includes:
#include "kinematics.h"
#include "rtapi_math.h"

// Add for kins based on genserkins:
// #include "genserkins.h" //includes gomath,hal

//**********************************************************************
// static local variables and functions go here

static int userk_inited = 0;

//**********************************************************************
int userkKinematicsSetup(const int   comp_id,
                         const char* coordinates,
                         kparms*     kp)
{
    rtapi_print("\nuserkKinematicsSetup:\n"
                  "   %s <%s> max_joints=%d allow_duplicates=%d\n\n",
                __FILE__,coordinates,
                kp->max_joints,kp->allow_duplicates);
    userk_inited = 1;
    return 0; // 0 ==> OK
}

int userkKinematicsForward(const double *joint,
                           struct EmcPose * world,
                           const KINEMATICS_FORWARD_FLAGS * fflags,
                           KINEMATICS_INVERSE_FLAGS * iflags)
{
    if (!userk_inited) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "userkKinematics: not initialized\n");
        return -1;
    }
    return identityKinematicsForward(joint,world,fflags,iflags);
}

int userkKinematicsInverse(const EmcPose * pos,
                           double *joint,
                           const KINEMATICS_INVERSE_FLAGS * iflags,
                           KINEMATICS_FORWARD_FLAGS * fflags)
{
    return identityKinematicsInverse(pos,joint,iflags,fflags);
}
