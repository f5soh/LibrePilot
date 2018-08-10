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

// Private constants

#define STACK_SIZE_BYTES 650


#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

// Private types

// Private variables
static xTaskHandle taskHandle;
static bool aeroEnabled  = false;
static AeroSettingsData aeroSettings;
static float nz_filtered = 1.0f;

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

    // Main task loop
    portTickType lastSysTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastSysTime, aeroSettings.MeasurementPeriod / portTICK_RATE_MS);

        // Get current raw Airspeed sensor data
        AirspeedSensorData airspeedSensor;
        AirspeedSensorGet(&airspeedSensor);

        // Get current Accel sensor data
        AccelStateData accelState;
        AccelStateGet(&accelState);

        VelocityStateData velocityState;
        VelocityStateGet(&velocityState);

        float mass_kg = (float)(aeroSettings.Mass) / 1000.0f;
        float wing_area_m2 = (float)(aeroSettings.WingSurface) / 100.0f;

        // From http://scherrer.pagesperso-orange.fr/matthieu/aero/papers/Which%20cl%20our%20model%20flies.pdf
        // M. Scherrer :  Which CLs is your model flying ?
        // The flight template explained to & applied to sailplane models : statiscal study of model's flights & optimization strategies.
        //
        // Estimated speed v1ms (m/s) @Cl=1
        // sqrt((2 * mass_kg * homeLocation.g_e) / (aeroSettings.Rho * wing_area_m2 * Cl))
        float v1ms = sqrt((2 * mass_kg * homeLocation.g_e) / (aeroSettings.Rho * wing_area_m2));
        aerostateData.V1ms  = v1ms; // m/s
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

        aerostateData.Airspeed = airspeedSensor.TrueAirspeed * 3.6f; // Km/h

        // Lift coefficient
        float cl = aerostateData.Nz.Current * ((v1ms * v1ms) / (airspeedSensor.TrueAirspeed * airspeedSensor.TrueAirspeed));
        aerostateData.Cl = boundf(cl, aeroSettings.ClLimits.Min, aeroSettings.ClLimits.Max);
        if (velocityState.Down > 0.2f) {
            aerostateData.GlideRatio = airspeedSensor.TrueAirspeed / velocityState.Down;
            aerostateData.Cd = aerostateData.Cl / aerostateData.GlideRatio;
        } else {
            aerostateData.GlideRatio = 0;
            aerostateData.Cd = 0;
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
