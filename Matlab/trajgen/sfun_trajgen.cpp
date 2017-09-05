#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  sfun_trajgen

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#define NPARAMS 7

#define SAMPLE_TIME_IDX 0
#define SAMPLE_TIME_PARAM(S) ssGetSFcnParam(S,SAMPLE_TIME_IDX)

#define NUMBER_OF_DOFS_IDX 1
#define NUMBER_OF_DOFS_PARAM(S) ssGetSFcnParam(S,NUMBER_OF_DOFS_IDX)

#define INIT_POS_IDX 2
#define INIT_POS_PARAM(S) ssGetSFcnParam(S,INIT_POS_IDX)

#define LIMIT_VEL_IDX 3
#define LIMIT_VEL_PARAM(S) ssGetSFcnParam(S,LIMIT_VEL_IDX)

#define LIMIT_ACC_IDX 4
#define LIMIT_ACC_PARAM(S) ssGetSFcnParam(S,LIMIT_ACC_IDX)

#define LIMIT_JRK_IDX 5
#define LIMIT_JRK_PARAM(S) ssGetSFcnParam(S,LIMIT_JRK_IDX)

#define SYNC_BEHAVIOR_IDX 6
#define SYNC_BEHAVIOR_PARAM(S) ssGetSFcnParam(S,SYNC_BEHAVIOR_IDX)


#define MDL_CHECK_PARAMETERS
#if defined(MATLAB_MEX_FILE)
// Function: mdlCheckParameters =============================================
// Abstract:
//    mdlCheckParameters verifies new parameter settings whenever parameter
//    change or are re-evaluated during a simulation. When a simulation is
//    running, changes to S-function parameters can occur at any time during
//    the simulation loop.
static void mdlCheckParameters(SimStruct *S)
{
    if (mxGetScalar(SAMPLE_TIME_PARAM(S)) < 0.0) {
        ssSetErrorStatus(S, "Sample time must be a positive scalar.");
        return;
    }
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    if (ndofs < 0) {
        ssSetErrorStatus(S, "Number of Dof's must be a positive integer.");
        return;
    }
    if (mxGetNumberOfElements(INIT_POS_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have an initial position defined.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_VEL_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have a velocity limit defined.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_ACC_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have an acceleration limit defined.");
        return;
    }
    if (mxGetNumberOfElements(LIMIT_JRK_PARAM(S)) != ndofs) {
        ssSetErrorStatus(S, "Each DoF must have a jerk limit defined.");
        return;
    }
}
#endif


// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 2;  // number of input ports
    int_T nOutputPorts = 3;  // number of output ports
    int_T needsInput   = 0;  // direct feed through
    
    ssSetNumSFcnParams(S, NPARAMS);  // Number of expected parameters
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
        return; // Simulink will report a parameter mismatch error
    }
    #endif
    ssSetSFcnParamTunable(S, SAMPLE_TIME_IDX, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, NUMBER_OF_DOFS_IDX, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, INIT_POS_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_VEL_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_ACC_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, LIMIT_JRK_IDX, SS_PRM_TUNABLE);
    ssSetSFcnParamTunable(S, SYNC_BEHAVIOR_IDX, SS_PRM_NOT_TUNABLE);
    
    // Register the number and type of states the S-Function uses
    ssSetNumContStates(S, 0);   // number of continuous states
    ssSetNumDiscStates(S, 0);   // number of discrete states
    
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    
    // Configure the input ports. First set the number of input ports.
    if (!ssSetNumInputPorts(S, nInputPorts)) return;
    if (!ssSetInputPortVectorDimension(S, 0, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 0, needsInput);
    if (!ssSetInputPortVectorDimension(S, 1, ndofs)) return;
    ssSetInputPortDirectFeedThrough(S, 1, needsInput);
    
    // Configure the output ports. First set the number of output ports.
    if (!ssSetNumOutputPorts(S,nOutputPorts)) return;
    if (!ssSetOutputPortVectorDimension(S, 0, ndofs)) return;
    if (!ssSetOutputPortVectorDimension(S, 1, ndofs)) return;
    if (!ssSetOutputPortVectorDimension(S, 2, ndofs)) return;
    
    ssSetNumSampleTimes(S, 1);  // number of sample times
    
    // Set size of the work vectors.
    ssSetNumRWork(S, 0);   // number of real work vector elements
    ssSetNumIWork(S, 0);   // number of integer work vector elements
    ssSetNumPWork(S, 4);   // number of pointer work vector elements
    ssSetNumModes(S, 0);   // number of mode work vector elements
    ssSetNumNonsampledZCs(S, 0);   // number of nonsampled zero crossings
    
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    
    #if defined(MATLAB_MEX_FILE)
    ssSetOptions(S, 
                 SS_OPTION_DISCRETE_VALUED_OUTPUT);
    #else
    ssSetOptions(S,
                 SS_OPTION_DISCRETE_VALUED_OUTPUT |
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE);
    #endif
}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(SAMPLE_TIME_PARAM(S)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


#define MDL_INITIALIZE_CONDITIONS
// Function: mdlInitializeConditions ========================================
// Abstract:
//    In this function, you should initialize the continuous and discrete
//    states for your S-function block.  The initial states are placed
//    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
//    You can also perform any other initialization activities that your
//    S-function may require. Note, this routine will be called at the
//    start of simulation and if it is present in an enabled subsystem
//    configured to reset states, it will be call when the enabled
//    subsystem restarts execution to reset the states.
static void mdlInitializeConditions(SimStruct *S)
{
    // Retrieve C++ object from the pointers vector
    RMLPositionInputParameters *IP = static_cast<RMLPositionInputParameters *>(ssGetPWork(S)[1]);
   
    // Set-up the state parameters
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    for (int_T dof=0; dof < ndofs; dof++)
    {
        IP->CurrentPositionVector->VecData[dof]     = mxGetPr(INIT_POS_PARAM(S))[dof];
        IP->CurrentVelocityVector->VecData[dof]     = 0.0;
        IP->CurrentAccelerationVector->VecData[dof] = 0.0;
        IP->MaxVelocityVector->VecData[dof]         = mxGetPr(LIMIT_VEL_PARAM(S))[dof];
        IP->MaxAccelerationVector->VecData[dof]     = mxGetPr(LIMIT_ACC_PARAM(S))[dof];
        IP->MaxJerkVector->VecData[dof]             = mxGetPr(LIMIT_JRK_PARAM(S))[dof];
        IP->TargetPositionVector->VecData[dof]      = mxGetPr(INIT_POS_PARAM(S))[dof];
        IP->TargetVelocityVector->VecData[dof]      = 0.0;
        IP->SelectionVector->VecData[dof]           = true;
    }
}


// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you have
//   states that should be initialized once, this is the place to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    const unsigned int ndofs = static_cast<unsigned int>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    ReflexxesAPI                *RML   = new ReflexxesAPI( ndofs, mxGetScalar(SAMPLE_TIME_PARAM(S)) );
    RMLPositionInputParameters  *IP    = new RMLPositionInputParameters( ndofs );
    RMLPositionOutputParameters *OP    = new RMLPositionOutputParameters( ndofs );
    RMLPositionFlags            *FLAGS = new RMLPositionFlags();
    
    const char* sync = mxArrayToString(SYNC_BEHAVIOR_PARAM(S));
    if (strcmp(sync,"NO_SYNCHRONIZATION") == 0){
        FLAGS->SynchronizationBehavior = RMLPositionFlags::NO_SYNCHRONIZATION;
    }
    else if (strcmp(sync,"PHASE_SYNCHRONIZATION_IF_POSSIBLE") == 0){
        FLAGS->SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    }
    else if (strcmp(sync,"ONLY_TIME_SYNCHRONIZATION") == 0){
        FLAGS->SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;
    }
    else if (strcmp(sync,"ONLY_PHASE_SYNCHRONIZATION") == 0){
        FLAGS->SynchronizationBehavior = RMLPositionFlags::ONLY_PHASE_SYNCHRONIZATION;
    }
    else {
        FLAGS->SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    }
    
    // Store new C++ object in the pointers vector
    ssGetPWork(S)[0] = RML;
    ssGetPWork(S)[1] = IP;
    ssGetPWork(S)[2] = OP;
    ssGetPWork(S)[3] = FLAGS;
}


// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Retrieve C++ object from the pointers vector
    RMLPositionOutputParameters *OP = static_cast<RMLPositionOutputParameters *>(ssGetPWork(S)[2]);
    
    // Get data addresses of I/O
    real_T *pos = ssGetOutputPortRealSignal(S, 0);
    real_T *vel = ssGetOutputPortRealSignal(S, 1);
    real_T *acc = ssGetOutputPortRealSignal(S, 2);
    
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    for (int_T dof=0; dof < ndofs; dof++)
    {
        pos[dof] = OP->NewPositionVector->VecData[dof];
        vel[dof] = OP->NewVelocityVector->VecData[dof];
        acc[dof] = OP->NewAccelerationVector->VecData[dof];
    }
}

#define MDL_UPDATE
// Function: mdlUpdate ======================================================
// Abstract:
//    This function is called once for every major integration time step.
//    Discrete states are typically updated here, but this function is
//    useful for performing any tasks that should only take place once per
//    integration step.
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // Retrieve C++ object from the pointers vector
    ReflexxesAPI                *RML   = static_cast<ReflexxesAPI *>(ssGetPWork(S)[0]);
    RMLPositionInputParameters  *IP    = static_cast<RMLPositionInputParameters *>(ssGetPWork(S)[1]);
    RMLPositionOutputParameters *OP    = static_cast<RMLPositionOutputParameters *>(ssGetPWork(S)[2]);
    RMLPositionFlags            *FLAGS = static_cast<RMLPositionFlags *>(ssGetPWork(S)[3]);
    
    // Get data addresses of I/O
    InputRealPtrsType target_pos_ptr = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType target_vel_ptr = ssGetInputPortRealSignalPtrs(S,1);
    
    // Set-up the input parameters
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    for (int_T dof=0; dof < ndofs; dof++)
    {
        IP->MaxVelocityVector->VecData[dof]     = mxGetPr(LIMIT_VEL_PARAM(S))[dof];
        IP->MaxAccelerationVector->VecData[dof] = mxGetPr(LIMIT_ACC_PARAM(S))[dof];
        IP->MaxJerkVector->VecData[dof]         = mxGetPr(LIMIT_JRK_PARAM(S))[dof];
        IP->TargetPositionVector->VecData[dof]  = *target_pos_ptr[dof];
        IP->TargetVelocityVector->VecData[dof]  = *target_vel_ptr[dof];
        IP->SelectionVector->VecData[dof]       = true;
    }
    
    // Calling the Reflexxes OTG algorithm
    int_T ResultValue = RML->RMLPosition( *IP, OP, *FLAGS );
    
    #if defined(MATLAB_MEX_FILE)
    if (ResultValue < 0)
    {
        static char msg[256];
        sprintf(msg,"An error occurred with Reflexxes OTG algorithm (%d).\n", ResultValue );
        ssSetErrorStatus(S,msg);
        return;
    }
    #endif
            
    // Feed the output values of the current control cycle back to
    // input values of the next control cycle
    *IP->CurrentPositionVector     = *OP->NewPositionVector;
    *IP->CurrentVelocityVector     = *OP->NewVelocityVector;
    *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
}


// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // Retrieve and destroy C++ objects
    ReflexxesAPI                *RML   = static_cast<ReflexxesAPI *>(ssGetPWork(S)[0]);
    RMLPositionInputParameters  *IP    = static_cast<RMLPositionInputParameters *>(ssGetPWork(S)[1]);
    RMLPositionOutputParameters *OP    = static_cast<RMLPositionOutputParameters *>(ssGetPWork(S)[2]);
    RMLPositionFlags            *FLAGS = static_cast<RMLPositionFlags *>(ssGetPWork(S)[3]);
    
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process
    delete  RML;
    delete  IP;
    delete  OP;
    delete  FLAGS;
}


#define MDL_RTW  /* Change to #undef to remove function */
#if defined(MATLAB_MEX_FILE)
// Function: mdlRTW =========================================================
// Abstract:
//
//    This function is called when the Real-Time Workshop is generating
//    the model.rtw file. In this method, you can call the following
//    functions which add fields to the model.rtw file.
static void mdlRTW(SimStruct *S)
{
    const int_T ndofs = static_cast<int_T>(mxGetScalar(NUMBER_OF_DOFS_PARAM(S)));
    
    // Parameter records for tunable S-function parameters.
    if (!ssWriteRTWParameters(S, 4,
            SSWRITE_VALUE_VECT,"InitPos", "",mxGetPr(INIT_POS_PARAM(S)),ndofs,
            SSWRITE_VALUE_VECT,"VelLimit","",mxGetPr(LIMIT_VEL_PARAM(S)),ndofs,
            SSWRITE_VALUE_VECT,"AccLimit","",mxGetPr(LIMIT_ACC_PARAM(S)),ndofs,
            SSWRITE_VALUE_VECT,"JrkLimit","",mxGetPr(LIMIT_JRK_PARAM(S)),ndofs) )
    {
        return;
    }
    
    // Parameter records for non-tunable S-function parameters.
    if (!ssWriteRTWParamSettings(S, 3,
            SSWRITE_VALUE_NUM,"SampleTime",mxGetScalar(SAMPLE_TIME_PARAM(S)),
            SSWRITE_VALUE_NUM,"NumberOfDoFs",mxGetScalar(NUMBER_OF_DOFS_PARAM(S)),
            SSWRITE_VALUE_STR,"SyncBehavior",mxArrayToString(SYNC_BEHAVIOR_PARAM(S)) ))
    {
        return;
    }
    
    // Work vector records for S-functions
    if (!ssWriteRTWWorkVect(S, "PWork", 4,
            "ReflexxesAPI", 1,
            "RMLPositionInputParameters", 1,
            "RMLPositionOutputParameters", 1,
            "RMLPositionFlags", 1) )
    {
        return;
    }
}
#endif /* MDL_RTW */


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism
#else
#include "cg_sfun.h"       // Code generation registration function
#endif
