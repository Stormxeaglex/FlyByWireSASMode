using System;
using UnityEngine;
using KSP;
using System.Collections;

namespace FlyByWireSASMode
{
    internal enum CustomSASMode
    {
        Stock,
        FlyByWire,
        FlyByWirePlaneMode,
        ParallelPos,
        ParallelNeg
    }

    internal class VesselState
    {
        public Vessel vessel;
        public CustomSASMode sasMode;
        public Vector3d direction;
        private Vector3d previousUp;

        public VesselState(Vessel vessel, CustomSASMode sasMode)
        {
            this.vessel = vessel;
            this.sasMode = sasMode;
            vessel.OnPreAutopilotUpdate += OnPreAutopilotUpdate;
            ResetDirection();
        }

        public void ResetDirection()
        {
            direction = vessel.GetTransform().up;
            previousUp = vessel.up;
        }

        public void Destroy()
        {
            vessel.OnPreAutopilotUpdate -= OnPreAutopilotUpdate;
            vessel.Autopilot.SetMode(vessel.autopilot.Mode);
        }

        private void OnPreAutopilotUpdate(FlightCtrlState st)
        {
            // continuously set mode to stability assist so we know which button to disable
            vessel.Autopilot.mode = VesselAutopilot.AutopilotMode.StabilityAssist;

            if (sasMode == CustomSASMode.FlyByWire || sasMode == CustomSASMode.FlyByWirePlaneMode)
            {
                // clear manual input
                st.pitch = 0f;
                st.yaw = 0f;

                if (FlightGlobals.ActiveVessel == vessel)
                {
                    // get input manually so this can work when under partial control
                    bool precisionMode = FlightInputHandler.fetch.precisionMode;

                    double pitch = GameSettings.AXIS_PITCH.GetAxis();
                    if (GameSettings.PITCH_DOWN.GetKey())
                        pitch = precisionMode ? -0.25 : -1.0;
                    else if (GameSettings.PITCH_UP.GetKey())
                        pitch = precisionMode ? 0.25 : 1.0;

                    double yaw = GameSettings.AXIS_YAW.GetAxis();
                    if (GameSettings.YAW_LEFT.GetKey())
                        yaw = precisionMode ? -0.25 : -1.0;
                    else if (GameSettings.YAW_RIGHT.GetKey())
                        yaw = precisionMode ? 0.25 : 1.0;

                    if (pitch != 0.0 || yaw != 0.0)
                    {
                        // increase speed when further away from control direction
                        float speed = Vector3.Dot(vessel.GetTransform().up, direction) - 1.2f;
                        speed *= FlyByWireSASMode.inputSensitivity;

                        // transform pitch/yaw input into a X/Y translation on the navball
                        QuaternionD navballAttitudeGymbal = FlyByWireSASMode.instance.navBall.attitudeGymbal;
                        direction =
                            QuaternionD.Inverse(navballAttitudeGymbal)
                            * Lib.QuaternionDFromEuler(pitch * -speed, yaw * -speed, 0.0)
                            * navballAttitudeGymbal
                            * direction;

                        direction.Normalize();
                    }
                }

                if (sasMode == CustomSASMode.FlyByWirePlaneMode)
                {
                    Quaternion horizonCorrection = Lib.QuaternionDFromToRotation(previousUp, vessel.up);
                    previousUp = vessel.up;
                    direction = horizonCorrection * direction;
                    direction.Normalize();
                }
            }
            else if (sasMode == CustomSASMode.ParallelPos || sasMode == CustomSASMode.ParallelNeg)
            {
                //Parallel SAS mode directions based on current speed display mode
                Vector3d? CalculatedSASVector = null;

                switch (FlightGlobals.speedDisplayMode)
                {
                    case FlightGlobals.SpeedDisplayModes.Surface:
                        CalculatedSASVector = GetSASHorizontalSurfaceVelocityDirection(vessel);
                        break;
                    case FlightGlobals.SpeedDisplayModes.Orbit:
                        break;
                    case FlightGlobals.SpeedDisplayModes.Target:
                        CalculatedSASVector = GetSASTargetForwardDirection(vessel);
                        break;
                }

                if (CalculatedSASVector.HasValue)
                    direction = CalculatedSASVector.Value;
            }

            // set SAS to follow requested direction
            vessel.Autopilot.SAS.lockedMode = false;
            vessel.Autopilot.SAS.SetTargetOrientation(direction, false);
        }


        private Vector3d? GetSASHorizontalSurfaceVelocityDirection(Vessel vessel)
        {
            Vector3d HorizontalVelocity = vessel.srf_velocity - Vector3d.Project(vessel.srf_velocity, vessel.up);

            if (HorizontalVelocity.sqrMagnitude <= 1e-5)
                return null;

            float sign = (sasMode == CustomSASMode.ParallelNeg) ? -1f : 1f;
            return sign * HorizontalVelocity.normalized;
        }

        private Vector3d? GetSASTargetForwardDirection(Vessel vessel)
        {
            Transform targetTransform = vessel.targetObject?.GetTransform();
            if (targetTransform == null)
                return null;
            
            Vector3d TargetForwardVector = (vessel.targetObject is Vessel) ? targetTransform.up : vessel.targetObject.GetFwdVector();
            float sign = (sasMode == CustomSASMode.ParallelNeg) ? -1f : 1f;
            return sign * TargetForwardVector;
        }

    }
}