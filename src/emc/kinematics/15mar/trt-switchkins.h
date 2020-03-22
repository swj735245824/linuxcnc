/********************************************************************
* License: GPL Version 2
* Copyright (c) 2009 All rights reserved.
*
* TRT table-rotary-tilting switchable kinematics
*
* Including files must #define:
*      TRT_KINS_NAME         component name
*      TRT_KINS_REQD_COORDS  mandatory coordinates
*      TRT_KINS_FORWARD      the Forward kinematics function
*      TRT_KINS_INVERSE      the Inverse kinematics function
*      TRT_KINS_PREFIX       hal pin prefix
*
* switchkins_type:
*   0 ==> identity (default)
*   1 ==> TRT_KINS_FORWARD,TRT_KINS_INVERSE
*   2
********************************************************************/

#ifndef TRTKINS_H // {
#define TRTKINS_H

#ifndef TRT_KINS_NAME
  #error  Missing TRT_KINS_NAME
#endif

#include "motion.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "kinematics.h"

//*********************************************************************
int trtkinsSetup(const char* compname,
                 const char* reqd_coordinates,
                 const char* coordinates,
                 const int   comp_id);

int xyzacKinematicsForward(const double *joints,
                           EmcPose * pos,
                           const KINEMATICS_FORWARD_FLAGS * fflags,
                           KINEMATICS_INVERSE_FLAGS * iflags);

int xyzacKinematicsInverse(const EmcPose * pos,
                           double *joints,
                           const KINEMATICS_INVERSE_FLAGS * iflags,
                           KINEMATICS_FORWARD_FLAGS * fflags);


int xyzbcKinematicsForward(const double *joints,
                           EmcPose * pos,
                           const KINEMATICS_FORWARD_FLAGS * fflags,
                           KINEMATICS_INVERSE_FLAGS * iflags);

int xyzbcKinematicsInverse(const EmcPose * pos,
                           double *joints,
                           const KINEMATICS_INVERSE_FLAGS * iflags,
                           KINEMATICS_FORWARD_FLAGS * fflags);

//*********************************************************************
static struct swdata {
    hal_bit_t   *kinstype_is_0;
    hal_bit_t   *kinstype_is_1;
    hal_bit_t   *kinstype_is_2;
} *swdata;

static hal_u32_t switchkins_type;
//*********************************************************************

int kinematicsSwitchable() {return 1;}

int kinematicsSwitch(int new_switchkins_type)
{
    switchkins_type = new_switchkins_type;
    switch (switchkins_type) {
        case 0: rtapi_print_msg(RTAPI_MSG_INFO,
                "kinematicsSwitch:Identity\n");
                *swdata->kinstype_is_0 = 1;
                *swdata->kinstype_is_1 = 0;
                *swdata->kinstype_is_2 = 0;
                break;
        case 1: rtapi_print_msg(RTAPI_MSG_INFO,
                "kinematicsSwitch: %s\n",TRT_KINS_NAME);
                *swdata->kinstype_is_1 = 1;
                *swdata->kinstype_is_0 = 0;
                *swdata->kinstype_is_2 = 0;
                break;
        case 2: rtapi_print_msg(RTAPI_MSG_INFO,
                "kinematicsSwitch:switchtype2\n");
                *swdata->kinstype_is_0 = 0;
                *swdata->kinstype_is_1 = 0;
                *swdata->kinstype_is_2 = 1;
                break;
       default: rtapi_print_msg(RTAPI_MSG_ERR,
                "kinematicsSwitch:BAD VALUE <%d>\n",
                switchkins_type);
                *swdata->kinstype_is_1 = 0;
                *swdata->kinstype_is_0 = 0;
                *swdata->kinstype_is_2 = 0;
                return -1; // FAIL
    }

    return 0; // 0==> no error
} // kinematicsSwitch()

int kinematicsForward(const double *joint,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    switch (switchkins_type) {
       case 0:    identityKinematicsForward(joint, pos, fflags, iflags);
               break;
       case 1:             TRT_KINS_FORWARD(joint, pos, fflags, iflags);
               break;
       case 2: switchtype2KinematicsForward(joint, pos, fflags, iflags);
               break;
      default: rtapi_print_msg(RTAPI_MSG_ERR,
                    "trtkins: Forward BAD switchkins_type </%d>\n",
                    switchkins_type);
               return -1;
    }
    return 0;
} // kinematicsForward()

int kinematicsInverse(const EmcPose * pos,
                      double *joint,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    switch (switchkins_type) {
       case 0:     identityKinematicsInverse(pos, joint, iflags, fflags);
                break;
       case 1:              TRT_KINS_INVERSE(pos, joint, iflags, fflags);
                break;
       case 2:  switchtype2KinematicsInverse(pos, joint, iflags, fflags);
                break;
       default: rtapi_print_msg(RTAPI_MSG_ERR,
                     "trtkins: Inverse BAD switchkins_type </%d>\n",
                     switchkins_type);
               return -1;
    }
    return 0;
} // kinematicsInverse()

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

//*********************************************************************
static char *coordinates = TRT_KINS_REQD_COORDS;
RTAPI_MP_STRING(coordinates, "Axes-to-joints-ordering");

EXPORT_SYMBOL(kinematicsSwitchable);
EXPORT_SYMBOL(kinematicsSwitch);
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int comp_id;
//*********************************************************************
int rtapi_app_main(void)
{
    #define DISALLOW_DUPLICATES 0
    int njoints = (int)strlen(TRT_KINS_REQD_COORDS);
    int res;

    if (njoints != strlen(coordinates)) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "ERROR %s: requires %d letters from set: <%s> Invalid: %s\n",
             TRT_KINS_NAME,njoints,TRT_KINS_REQD_COORDS,coordinates);
        goto error;
    }

    comp_id = hal_init(TRT_KINS_NAME);
    if(comp_id < 0) goto error;

    swdata = hal_malloc(sizeof(struct swdata));
    if (!swdata) goto error;

    if((res = hal_pin_bit_new("kinstype.is-0",
              HAL_OUT, &(swdata->kinstype_is_0), comp_id)) < 0) goto error;
    if((res = hal_pin_bit_new("kinstype.is-1",
              HAL_OUT, &(swdata->kinstype_is_1), comp_id)) < 0) goto error;
    if((res = hal_pin_bit_new("kinstype.is-2",
              HAL_OUT, &(swdata->kinstype_is_2), comp_id)) < 0) goto error;

    switchkins_type = 0; // startup with default type
    kinematicsSwitch(switchkins_type);

    if (identityKinematicsSetup(coordinates,
                                njoints,
                                DISALLOW_DUPLICATES)) goto error;
       
    if (trtkinsSetup(TRT_KINS_PREFIX,
                     TRT_KINS_REQD_COORDS,
                     coordinates,
                     comp_id)) goto error;

    if (switchtype2KinematicsSetup(coordinates,
                                   njoints,
                                   comp_id)) goto error;

    hal_ready(comp_id);
    return 0;

error:
    rtapi_print_msg(RTAPI_MSG_ERR,"%s app FAIL\n",TRT_KINS_NAME);
    hal_exit(comp_id);
    return -1;
} // rtapi_app_main()

void rtapi_app_exit(void) { hal_exit(comp_id); }

#endif // }
