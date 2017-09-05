/*
 * trajgen2.h
 *
 * Code generation for model "trajgen2".
 *
 * Model version              : 1.296
 * Simulink Coder version : 8.10 (R2016a) 10-Feb-2016
 * C++ source code generated on : Thu Apr 06 16:47:58 2017
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_trajgen2_h_
#define RTW_HEADER_trajgen2_h_
#include <cmath>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef trajgen2_COMMON_INCLUDES_
# define trajgen2_COMMON_INCLUDES_
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* trajgen2_COMMON_INCLUDES_ */

#include "trajgen2_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T y;                            /* '<S1>/MATLAB Function' */
  boolean_T Compare;                   /* '<S6>/Compare' */
  boolean_T Uk1;                       /* '<S3>/Delay Input1' */
  boolean_T FixPtRelationalOperator;   /* '<S3>/FixPt Relational Operator' */
} B_trajgen2_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T EnabledDelay_DSTATE[20];      /* '<S1>/Enabled Delay' */
  struct {
    void *ReflexxesAPI;
    void *RMLPositionInputParameters;
    void *RMLPositionOutputParameters;
    void *RMLPositionFlags;
  } ReflexxesTrajectoryGenerator_PW;   /* '<S1>/Reflexxes Trajectory Generator' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace1_PWORK;              /* '<Root>/From Workspace1' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace2_PWORK;              /* '<Root>/From Workspace 2' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace3_PWORK;              /* '<Root>/From Workspace 3' */

  struct {
    int_T PrevIndex;
  } FromWorkspace1_IWORK;              /* '<Root>/From Workspace1' */

  struct {
    int_T PrevIndex;
  } FromWorkspace2_IWORK;              /* '<Root>/From Workspace 2' */

  struct {
    int_T PrevIndex;
  } FromWorkspace3_IWORK;              /* '<Root>/From Workspace 3' */

  boolean_T DelayInput1_DSTATE;        /* '<S3>/Delay Input1' */
} DW_trajgen2_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T s;                            /* '<Root>/s' */
  real_T DelayedTarget;                /* '<Root>/Delayed Target ' */
  real_T sp;                           /* '<Root>/sp' */
  real_T spp;                          /* '<Root>/spp' */
  real_T dist;                         /* '<Root>/dist' */
  real_T NominalTarget;                /* '<Root>/Nominal Target' */
} ExtY_trajgen2_T;

/* Parameters (auto storage) */
struct P_trajgen2_T_ {
  real_T min_dist;                     /* Variable: min_dist
                                        * Referenced by: '<Root>/robot_trajectory_s1'
                                        */
  real_T robot_trajectory_data[300];   /* Variable: robot_trajectory_data
                                        * Referenced by: '<Root>/robot_trajectory_data'
                                        */
  real_T robot_trajectory_s[100];      /* Variable: robot_trajectory_s
                                        * Referenced by: '<Root>/robot_trajectory_s'
                                        */
  real_T ReflexxesTrajectoryGenerator_am;/* Mask Parameter: ReflexxesTrajectoryGenerator_am
                                          * Referenced by: '<S1>/Reflexxes Trajectory Generator'
                                          */
  real_T ReflexxesTrajectoryGenerator_jm;/* Mask Parameter: ReflexxesTrajectoryGenerator_jm
                                          * Referenced by: '<S1>/Reflexxes Trajectory Generator'
                                          */
  real_T ReflexxesTrajectoryGenerator_pi;/* Mask Parameter: ReflexxesTrajectoryGenerator_pi
                                          * Referenced by: '<S1>/Reflexxes Trajectory Generator'
                                          */
  real_T ReflexxesTrajectoryGenerator_vm;/* Mask Parameter: ReflexxesTrajectoryGenerator_vm
                                          * Referenced by: '<S1>/Reflexxes Trajectory Generator'
                                          */
  boolean_T DetectFallNonpositive_vinit;/* Mask Parameter: DetectFallNonpositive_vinit
                                         * Referenced by: '<S3>/Delay Input1'
                                         */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S6>/Constant'
                                        */
  real_T EnabledDelay_InitialCondition;/* Expression: 0.0
                                        * Referenced by: '<S1>/Enabled Delay'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S1>/Constant1'
                                        */
  uint32_T EnabledDelay_DelayLength;   /* Computed Parameter: EnabledDelay_DelayLength
                                        * Referenced by: '<S1>/Enabled Delay'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_trajgen2_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_trajgen2_T trajgen2_P;

#ifdef __cplusplus

}
#endif

/* Block signals (auto storage) */
extern B_trajgen2_T trajgen2_B;

/* Block states (auto storage) */
extern DW_trajgen2_T trajgen2_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* External outputs (root outports fed by signals with auto storage) */
  extern ExtY_trajgen2_T trajgen2_Y;

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void trajgen2_initialize(void);
  extern void trajgen2_step(void);
  extern void trajgen2_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_trajgen2_T *const trajgen2_M;

#ifdef __cplusplus

}
#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'trajgen2'
 * '<S1>'   : 'trajgen2/Subsystem'
 * '<S2>'   : 'trajgen2/Subsystem/Detect Fall Negative'
 * '<S3>'   : 'trajgen2/Subsystem/Detect Fall Nonpositive'
 * '<S4>'   : 'trajgen2/Subsystem/MATLAB Function'
 * '<S5>'   : 'trajgen2/Subsystem/Detect Fall Negative/Negative'
 * '<S6>'   : 'trajgen2/Subsystem/Detect Fall Nonpositive/Nonpositive'
 */
#endif                                 /* RTW_HEADER_trajgen2_h_ */
