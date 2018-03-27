/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import "ViewController.h"
#import "ViewController+Camera.h"
#import "ViewController+Sensor.h"
#import "ViewController+SLAM.h"
#import "ViewController+OpenGL.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

@implementation ViewController (Sensor)

#pragma mark -  Structure Sensor delegates

- (void)setupStructureSensor
{
    // Get the sensor controller singleton
    _sensorController = [STSensorController sharedController];
    
    // Set ourself as the delegate to receive sensor data.
    _sensorController.delegate = self;
}

- (BOOL)isStructureConnectedAndCharged
{
    return [_sensorController isConnected] && ![_sensorController isLowPower];
}

- (void)sensorDidConnect
{
    NSLog(@"[Structure] Sensor connected!");
    
    if ([self currentStateNeedsSensor])
        [self connectToStructureSensorAndStartStreaming];
}

- (void)sensorDidLeaveLowPowerMode
{
    _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
    [self updateAppStatusMessage];
}

- (void)sensorBatteryNeedsCharging
{
    // Notify the user that the sensor needs to be charged.
    _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToCharge;
    [self updateAppStatusMessage];
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    if (reason == STSensorControllerDidStopStreamingReasonAppWillResignActive)
    {
        [self stopColorCamera];
        NSLog(@"[Structure] Stopped streaming because the app will resign its active state.");
    }
    else
    {
        NSLog(@"[Structure] Stopped streaming for an unknown reason.");
    }
}

- (void)sensorDidDisconnect
{
    // If we receive the message while in background, do nothing. We'll check the status when we
    // become active again.
    if ([[UIApplication sharedApplication] applicationState] != UIApplicationStateActive)
        return;
    
    NSLog(@"[Structure] Sensor disconnected!");
    
    // Reset the scan on disconnect, since we won't be able to recover afterwards.
    if (_slamState.scannerState == ScannerStateScanning)
    {
        [self resetButtonPressed:self];
    }
    
    if (_useColorCamera)
        [self stopColorCamera];
    
    // We only show the app status when we need sensor
    if ([self currentStateNeedsSensor])
    {
        _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
        [self updateAppStatusMessage];
    }
    
    if (_calibrationOverlay)
        _calibrationOverlay.hidden = true;
    
    [self updateIdleTimer];
}


- (STSensorControllerInitStatus)connectToStructureSensorAndStartStreaming
{
    
    // Try connecting to a Structure Sensor.
    STSensorControllerInitStatus result = [_sensorController initializeSensorConnection];
    
    if (result == STSensorControllerInitStatusSuccess || result == STSensorControllerInitStatusAlreadyInitialized)
    {
        // Even though _useColorCamera was set in viewDidLoad by asking if an approximate calibration is guaranteed,
        // it's still possible that the Structure Sensor that has just been plugged in has a custom or approximate calibration
        // that we couldn't have known about in advance.
        
        STCalibrationType calibrationType = [_sensorController calibrationType];

        _useColorCamera =
               calibrationType == STCalibrationTypeApproximate
            || calibrationType == STCalibrationTypeDeviceSpecific;

        if (_useColorCamera)
        {
            // Leave _dynamicOptions.newTrackerIsOn alone. It may have been modified by the user.

            // Enable the new tracker UI switch, since both depth and color frames can be captured.
            _dynamicOptions.newTrackerSwitchEnabled = true;

            // Leave _dynamicOptions.highResColoring alone. It may have been modified by the user.
            
            // Enable the high-res coloring UI switch when high-resolution color capture is available.
            _dynamicOptions.highResColoringSwitchEnabled = [self videoDeviceSupportsHighResColor];
        }
        else
        {
            // Disable both the new tracker and its UI switch, since there is no color camera input.
            _dynamicOptions.newTrackerSwitchEnabled = false;
            _dynamicOptions.newTrackerIsOn = false;

            // Disable both the high resolution coloring and its UI switch, since there is no color camera input.
            _dynamicOptions.highResColoring = false;
            _dynamicOptions.highResColoringSwitchEnabled = false;

            // If we can't use the color camera, then don't try to use registered depth.
            _options.useHardwareRegisteredDepth = false;
        }

        // Make sure the new mapper and high-resolution mapping switches are always enabled.
        _dynamicOptions.newMapperSwitchEnabled = true;
        _dynamicOptions.highResMappingSwitchEnabled = true;

        // Reset the SLAM pipeline.
        // This will also synchronize the UI switches states from the dynamic option values.
        [self onSLAMOptionsChanged];

        // Update the app status message.
        _appStatus.sensorStatus = AppStatus::SensorStatusOk;
        [self updateAppStatusMessage];
        
        // Start streaming depth data.
        [self startStructureSensorStreaming];
    }
    else
    {
        switch (result)
        {
            case STSensorControllerInitStatusSensorNotFound:
                NSLog(@"[Structure] No sensor found"); break;
            case STSensorControllerInitStatusOpenFailed:
                NSLog(@"[Structure] Error: Open failed."); break;
            case STSensorControllerInitStatusSensorIsWakingUp:
                NSLog(@"[Structure] Error: Sensor still waking up."); break;
            default: {}
        }
        
        _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
        [self updateAppStatusMessage];
    }
    
    [self updateIdleTimer];
    
    return result;
}

- (void)startStructureSensorStreaming
{
    if (![self isStructureConnectedAndCharged])
        return;
    
    // Tell the driver to start streaming.
    NSError *error = nil;
    BOOL optionsAreValid = FALSE;
    if (_useColorCamera)
    {
        // We can use either registered or unregistered depth.
        _structureStreamConfig = _options.useHardwareRegisteredDepth ? STStreamConfigRegisteredDepth320x240 : STStreamConfigDepth320x240;
        
        if (_options.useHardwareRegisteredDepth)
        {
            // We are using the color camera, so let's make sure the depth gets synchronized with it.
            // If we use registered depth, we also need to specify a fixed lens position value for the color camera.
            optionsAreValid = [_sensorController startStreamingWithOptions:@{kSTStreamConfigKey : @(_structureStreamConfig),
                                                                             kSTFrameSyncConfigKey : @(STFrameSyncDepthAndRgb),
                                                                             kSTColorCameraFixedLensPositionKey: @(_options.lensPosition)}
                                                                     error:&error];
        }
        else
        {
            // We are using the color camera, so let's make sure the depth gets synchronized with it.
            optionsAreValid = [_sensorController startStreamingWithOptions:@{kSTStreamConfigKey : @(_structureStreamConfig),
                                                                             kSTFrameSyncConfigKey : @(STFrameSyncDepthAndRgb)}
                                                                     error:&error];
        }
        
        [self startColorCamera];
    }
    else
    {
        _structureStreamConfig = STStreamConfigDepth320x240;
        
        optionsAreValid = [_sensorController startStreamingWithOptions:@{kSTStreamConfigKey : @(_structureStreamConfig),
                                                                         kSTFrameSyncConfigKey : @(STFrameSyncOff)} error:&error];
    }
    
    if (!optionsAreValid)
    {
        NSLog(@"Error during streaming start: %s", [[error localizedDescription] UTF8String]);
        return;
    }
    
    NSLog(@"[Structure] Streaming started.");
    
    // Notify and initialize streaming dependent objects.
    [self onStructureSensorStartedStreaming];
}

- (void)onStructureSensorStartedStreaming
{
    STCalibrationType calibrationType = [_sensorController calibrationType];
    
    // The Calibrator app will be updated to support future iPads, and additional attachment brackets will be released as well.
    const bool deviceIsLikelySupportedByCalibratorApp = (UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPad);
    
    // Only present the option to switch over to the Calibrator app if the sensor doesn't already have a device specific
    // calibration and the app knows how to calibrate this iOS device.
    if (calibrationType != STCalibrationTypeDeviceSpecific && deviceIsLikelySupportedByCalibratorApp)
    {
        if (!_calibrationOverlay)
            _calibrationOverlay = [CalibrationOverlay calibrationOverlaySubviewOf:self.view atOrigin:CGPointMake(8, 8)];
        else
            _calibrationOverlay.hidden = false;
    }
    else
    {
        if (_calibrationOverlay)
            _calibrationOverlay.hidden = true;
    }
    
    if (!_slamState.initialized)
        [self setupSLAM];
}

- (void)sensorDidOutputSynchronizedDepthFrame:(STDepthFrame*)depthFrame
                                   colorFrame:(STColorFrame*)colorFrame
{
    if (_slamState.initialized)
    {
        [self processDepthFrame:depthFrame colorFrameOrNil:colorFrame];
        // Scene rendering is triggered by new frames to avoid rendering the same view several times.
        [self renderSceneForDepthFrame:depthFrame colorFrameOrNil:colorFrame];
    }
}

- (void)sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    if (_slamState.initialized)
    {
        [self processDepthFrame:depthFrame colorFrameOrNil:nil];
        // Scene rendering is triggered by new frames to avoid rendering the same view several times.
        [self renderSceneForDepthFrame:depthFrame colorFrameOrNil:nil];
    }
}

@end
