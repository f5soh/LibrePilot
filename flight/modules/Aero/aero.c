/**
 ******************************************************************************
 *
 * @file       aero.c
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2018.
 * @brief      aero module. Estimate Fixed Wing aerodynamics
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include <openpilot.h>
#include "inc/aero.h"
#include <CoordinateConversions.h>
#include <aerosettings.h>
#include <aerostate.h>
#include <aerocl.h>
#include <barosensor.h>
#include <airspeedsensor.h>
#include <accelstate.h>
#include <accessorydesired.h>
#include <attitudestate.h>
#include <callbackinfo.h>
#include <flightstatus.h>
#include <gpstime.h>
#include <homelocation.h>
#include <hwsettings.h>
#include <positionstate.h>
#include <velocitystate.h>
#include "taskinfo.h"

#ifdef PIOS_INCLUDE_DEBUGLOG
#include "debuglogsettings.h"
#include "debuglogcontrol.h"
#include "debuglogstatus.h"
#include "debuglogentry.h"
#endif /* PIOS_INCLUDE_DEBUGLOG */

// Private constants

#define STACK_SIZE_BYTES 1000


#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

// Private types

// Private variables
static xTaskHandle taskHandle;
static bool aeroEnabled  = false;
static AeroSettingsData aeroSettings;
static float nz_filtered = 1.0f;
static float airspeed_filtered = 1.0f;

// Private functions
static void aeroTask(void *parameters);
static void AeroSettingsUpdatedCb(UAVObjEvent *ev);


/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AeroStart()
{
    // Check if module is enabled or not
    if (aeroEnabled == false) {
        return -1;
    }

    // Start main task
    xTaskCreate(aeroTask, "Aero", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_AERO, taskHandle);
    return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t AeroInitialize()
{
    HwSettingsOptionalModulesData optionalModules;

    HwSettingsOptionalModulesGet(&optionalModules);

#ifdef MODULE_AERO_BUILTIN
    aeroEnabled = true;
    optionalModules.Aero = HWSETTINGS_OPTIONALMODULES_ENABLED;
    HwSettingsOptionalModulesSet(&optionalModules);
#else
    if (optionalModules.Aero == HWSETTINGS_OPTIONALMODULES_ENABLED) {
        aeroEnabled = true;
    } else {
        aeroEnabled = false;
        return -1;
    }
#endif

    AeroStateInitialize();
    AeroClInitialize();

    AeroSettingsConnectCallback(AeroSettingsUpdatedCb);

    return 0;
}
MODULE_INITCALL(AeroInitialize, AeroStart);


/**
 * Module thread, should not return.
 */
static void aeroTask(__attribute__((unused)) void *parameters)
{
    AeroSettingsUpdatedCb(AeroSettingsHandle());

    AeroStateData aerostateData;
    AeroStateGet(&aerostateData);

    AeroSettingsUpdatedCb(NULL);

    HomeLocationData homeLocation;
    HomeLocationGet(&homeLocation);

    AccessoryDesiredData accessoryValue;
    AeroClData data;

    bool docalc = false;
    bool debuglog_done = true;
    int publishedCountersInstances = 0;

#ifdef PIOS_INCLUDE_DEBUGLOG
    static DebugLogSettingsData settings;
    uint16_t instId = 0;
    GPSTimeData time;
#endif /* PIOS_INCLUDE_DEBUGLOG */

    // Main task loop
    portTickType lastSysTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastSysTime, aeroSettings.MeasurementPeriod / portTICK_RATE_MS);

        int needed_instances = 1 + (aeroSettings.ClLimits.Max - aeroSettings.ClLimits.Min) / aeroSettings.ClLimits.Resolution;

        // Spend some loops and create Cl counters
        if (publishedCountersInstances < needed_instances) {
            if (publishedCountersInstances != 0) {
                AeroClCreateInstance();
            }
            data.Cl = aeroSettings.ClLimits.Min + (publishedCountersInstances * aeroSettings.ClLimits.Resolution);
            data.ClCount = 0; // roundf((data.Cl / aeroSettings.ClLimits.Resolution) + fabsf(aeroSettings.ClLimits.Min / aeroSettings.ClLimits.Resolution));

            AeroClInstSet(publishedCountersInstances, &data);
            publishedCountersInstances++;
        }

        // Get current raw Airspeed sensor data
        AirspeedSensorData airspeedSensor;
        AirspeedSensorGet(&airspeedSensor);

        // Get current Accel sensor data
        AccelStateData accelState;
        AccelStateGet(&accelState);

        VelocityStateData velocityState;
        VelocityStateGet(&velocityState);

        PositionStateData positionState;
        PositionStateGet(&positionState);

        if (aeroSettings.StartCalcSource == AEROSETTINGS_STARTCALCSOURCE_DISABLED) {
            docalc = true;
        } else {
            AccessoryDesiredInstGet(aeroSettings.StartCalcSource - 1, &accessoryValue);
            if (!docalc && (accessoryValue.AccessoryVal > 0.2f)) {
                // Accessory Max
                // Reset Min/max values and start estimation
                aerostateData.Nz.Max = aerostateData.Nz.Current;
                aerostateData.Nz.Min = aerostateData.Nz.Current;
                aerostateData.Airspeed.Max = aerostateData.Airspeed.Current;
                aerostateData.Airspeed.Min = aerostateData.Airspeed.Current;
                docalc = true;
                debuglog_done = true;
            } else if (accessoryValue.AccessoryVal < -0.2f) {
                // Accessory Min
                docalc = false;
                debuglog_done = false;
            } else if (!debuglog_done) {
#ifdef PIOS_INCLUDE_DEBUGLOG
                // Accessory Middle : store flight template from UAVO AeroCl to onboard flash
                if (instId == 0) {
                    // Enable log
                    DebugLogSettingsGet(&settings);
                    settings.LoggingEnabled = DEBUGLOGSETTINGS_LOGGINGENABLED_ALWAYS;
                    DebugLogSettingsSet(&settings);
                }

                UAVObjInstanceLogging(AeroClHandle(), instId);
                instId++;

                if (instId > UAVObjGetNumInstances(AeroClHandle())) {
                    debuglog_done = true;
                    // Reset for future storage process
                    instId = 0;

                    // Add time info to log
                    GPSTimeGet(&time);
                    if (time.Year > 2000) {
                        PIOS_DEBUGLOG_Printf("Flight Template on %d/%d/%d at %d:%d", time.Day, time.Month, time.Year, time.Hour, time.Minute);
                    }
                    // Disable log
                    settings.LoggingEnabled = DEBUGLOGSETTINGS_LOGGINGENABLED_DISABLED;
                    DebugLogSettingsSet(&settings);
                }
#endif /* PIOS_INCLUDE_DEBUGLOG */
            }
        }

        float mass_kg = (float)(aeroSettings.Model.Mass) / 1000.0f;
        float wing_area_m2 = (float)(aeroSettings.Model.WingSurface) / 100.0f;

        // From http://scherrer.pagesperso-orange.fr/matthieu/aero/papers/Which%20cl%20our%20model%20flies.pdf
        // M. Scherrer :  Which CLs is your model flying ?
        // The flight template explained to & applied to sailplane models : statiscal study of model's flights & optimization strategies.
        //
        // Estimated speed v1ms (m/s) @Cl=1
        // sqrt((2 * mass_kg * homeLocation.g_e) / (aeroSettings.Rho * wing_area_m2 * Cl))
        float v1ms = sqrt((2 * mass_kg * homeLocation.g_e) / (aeroSettings.Rho * wing_area_m2));
        aerostateData.V1kmh = v1ms * 3.6f; // km/h

        // Normal Acceleration (G unit)
        nz_filtered = ((1 - aeroSettings.NzLowPassAlpha) * (-accelState.z / homeLocation.g_e)) + (nz_filtered * (aeroSettings.NzLowPassAlpha));
        aerostateData.Nz.Current = boundf(nz_filtered, aeroSettings.NzLimits.Min, aeroSettings.NzLimits.Max);

        // Update Nz Min/Max
        if (aerostateData.Nz.Current > aerostateData.Nz.Max) {
            aerostateData.Nz.Max = aerostateData.Nz.Current;
        }
        if (aerostateData.Nz.Current < aerostateData.Nz.Min) {
            aerostateData.Nz.Min = aerostateData.Nz.Current;
        }

        // Airspeed
        // Choose TrueAirSpeed if available
        float airspeed = (airspeedSensor.TrueAirspeed > 0) ? airspeedSensor.TrueAirspeed : airspeedSensor.CalibratedAirspeed;
        airspeed_filtered = ((1 - aeroSettings.AirSpeedLowPassAlpha) * (airspeed * 3.6f)) + (airspeed_filtered * (aeroSettings.AirSpeedLowPassAlpha));
        aerostateData.Airspeed.Current = airspeed_filtered; // Km/h
        aerostateData.Temperature = airspeedSensor.Temperature - 273.15f; // °Celsius

        // Update Airspeed Min/Max
        if (aerostateData.Airspeed.Current > aerostateData.Airspeed.Max) {
            aerostateData.Airspeed.Max = aerostateData.Airspeed.Current;
        }
        if (aerostateData.Airspeed.Current < aerostateData.Airspeed.Min) {
            aerostateData.Airspeed.Min = aerostateData.Airspeed.Current;
        }

        // Vx/Vz Airspeed/Sink polar
        // Ax^2 + Bx + C = y
        // Convert input sink values (m/s) to km/h
        float vz0kmh = aeroSettings.Vz[0] * 3.6f;
        float vz1kmh = aeroSettings.Vz[1] * 3.6f;
        float vz2kmh = aeroSettings.Vz[2] * 3.6f;

        // Airspeed values
        float vx0    = aeroSettings.Vx[0];
        float vx1    = aeroSettings.Vx[1];
        float vx2    = aeroSettings.Vx[2];

        // Sink polar coefficients calculation
        float a = ((vx1 - vx2) * (vz0kmh - vz2kmh) + (vx2 - vx0) * (vz1kmh - vz2kmh)) / ((vx0 * vx0) * (vx1 - vx2) + (vx1 * vx1) * (vx2 - vx0) + (vx2 * vx2) * (vx0 - vx1));
        float b = (vz1kmh - vz2kmh - a * ((vx1 * vx1) - (vx2 * vx2))) / (vx1 - vx2);
        float c = vz2kmh - a * (vx2 * vx2) - b * vx2;
        aerostateData.VzPolarDef.a = a;
        aerostateData.VzPolarDef.b = b;
        aerostateData.VzPolarDef.c = c;

        // Potential energy
        // m * g * h
        float Epot = mass_kg * homeLocation.g_e * (-positionState.Down);

        // Kinetic energy
        // 0.5 * m * v²
        float Ekin = 0.5f * mass_kg * ((airspeed_filtered * airspeed_filtered));

        aerostateData.TotalEnergy = Epot + Ekin;

        // Reference sink in m/s, for current airspeed (filtered)
        aerostateData.VzRef = ((a * airspeed_filtered * airspeed_filtered) + (b * airspeed_filtered) + c) / 3.6f;

        if (docalc && (UAVObjGetNumInstances(AeroClHandle()) == needed_instances)) {
            // Lift coefficient
            float cl     = aerostateData.Nz.Current * ((v1ms * v1ms) / (airspeed_filtered * airspeed_filtered));
            aerostateData.Cl = boundf(cl, aeroSettings.ClLimits.Min, aeroSettings.ClLimits.Max);
            int cl_index = roundf((aerostateData.Cl / aeroSettings.ClLimits.Resolution) + fabsf(aeroSettings.ClLimits.Min / aeroSettings.ClLimits.Resolution));
            AeroClInstGet(cl_index, &data);
            data.ClCount++;
            AeroClInstSet(cl_index, &data);
            if (velocityState.Down > 0.2f) {
                aerostateData.GlideRatio = airspeed_filtered / velocityState.Down;
                aerostateData.Cd = aerostateData.Cl / aerostateData.GlideRatio;
            } else {
                aerostateData.Cd = 0;
                aerostateData.GlideRatio = 0;
            }
        } else {
            aerostateData.Cl = 0;
        }

        // Set the UAVO
        AeroStateSet(&aerostateData);
    }
}


static void AeroSettingsUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    AeroSettingsGet(&aeroSettings);
}


/**
 * @}
 * @}
 */
