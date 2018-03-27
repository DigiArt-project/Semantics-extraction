/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import "ViewController.h"
#import "ViewController+OpenGL.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#pragma mark - Utilities

namespace // anonymous namespace for local functions
{
    float deltaRotationAngleBetweenPosesInDegrees (const GLKMatrix4& previousPose, const GLKMatrix4& newPose)
    {
        GLKMatrix4 deltaPose = GLKMatrix4Multiply(newPose,
                                                  // Transpose is equivalent to inverse since we will only use the rotation part.
                                                  GLKMatrix4Transpose(previousPose));
        
        // Get the rotation component of the delta pose
        GLKQuaternion deltaRotationAsQuaternion = GLKQuaternionMakeWithMatrix4(deltaPose);
        
        // Get the angle of the rotation
        const float angleInDegree = GLKQuaternionAngle(deltaRotationAsQuaternion)/M_PI*180;
        
        return angleInDegree;
    }

    NSString* computeTrackerMessage (STTrackerHints hints)
    {
        if (hints.trackerIsLost)
            return @"Tracking Lost! Please Realign or Press Reset.";
        
        if (hints.modelOutOfView)
            return @"Please put the model back in view.";
        
        if (hints.sceneIsTooClose)
            return @"Too close to the scene! Please step back.";

        return nil;
    }
}

@implementation ViewController (SLAM)

#pragma mark - SLAM

// Set up SLAM related objects.
- (void)setupSLAM
{
    if (_slamState.initialized)
        return;
    
    // Initialize the scene.
    _slamState.scene = [[STScene alloc] initWithContext:_display.context
                                      freeGLTextureUnit:GL_TEXTURE2];
    
    // Initialize the camera pose tracker.
    NSDictionary* trackerOptions = @{
                                     kSTTrackerTypeKey: _dynamicOptions.newTrackerIsOn ? @(STTrackerDepthAndColorBased) : @(STTrackerDepthBased),
                                     kSTTrackerTrackAgainstModelKey: @TRUE, // tracking against the model is much better for close range scanning.
                                     kSTTrackerQualityKey: @(STTrackerQualityAccurate),
                                     kSTTrackerBackgroundProcessingEnabledKey: @YES
                                     };
    
    // Initialize the camera pose tracker.
    _slamState.tracker = [[STTracker alloc] initWithScene:_slamState.scene options:trackerOptions];
    
    // Default volume size set in options struct
    if( isnan(_slamState.volumeSizeInMeters.x) )
        _slamState.volumeSizeInMeters = _options.initVolumeSizeInMeters;
    
    // The mapper will be initialized when we start scanning.
    
    // Setup the cube placement initializer.
    _slamState.cameraPoseInitializer = [[STCameraPoseInitializer alloc]
                                        initWithVolumeSizeInMeters:_slamState.volumeSizeInMeters
                                        options:@{kSTCameraPoseInitializerStrategyKey: @(STCameraPoseInitializerStrategyTableTopCube)}];
    
    // Set up the cube renderer with the current volume size.
    _display.cubeRenderer = [[STCubeRenderer alloc] initWithContext:_display.context];
    
    // Set up the initial volume size.
    [self adjustVolumeSize:_slamState.volumeSizeInMeters];
    
    // Start with cube placement mode
    [self enterCubePlacementState];
    
    NSDictionary* keyframeManagerOptions = @{
                                             kSTKeyFrameManagerMaxSizeKey: @(_options.maxNumKeyFrames),
                                             kSTKeyFrameManagerMaxDeltaTranslationKey: @(_options.maxKeyFrameTranslation),
                                             kSTKeyFrameManagerMaxDeltaRotationKey: @(_options.maxKeyFrameRotation), // 20 degrees.
                                             };
    
    _slamState.keyFrameManager = [[STKeyFrameManager alloc] initWithOptions:keyframeManagerOptions];
    
    _depthAsRgbaVisualizer = [[STDepthToRgba alloc] initWithOptions:@{kSTDepthToRgbaStrategyKey: @(STDepthToRgbaStrategyGray)}];
    
    _slamState.initialized = true;
}

- (void)resetSLAM
{
    _slamState.prevFrameTimeStamp = -1.0;
    [_slamState.mapper reset];
    [_slamState.tracker reset];
    [_slamState.scene clear];
    [_slamState.keyFrameManager clear];
    
    [self enterCubePlacementState];
}

- (void)clearSLAM
{
    _slamState.initialized = false;
    _slamState.scene = nil;
    _slamState.tracker = nil;
    _slamState.mapper = nil;
    _slamState.keyFrameManager = nil;
}

- (void)setupMapper
{
    if (_slamState.mapper)
        _slamState.mapper = nil; // make sure we first remove a previous mapper.
    
    // Here, we set a larger volume bounds size when mapping in high resolution.
    const float lowResolutionVolumeBounds = 125;
    const float highResolutionVolumeBounds = 200;
    
    float voxelSizeInMeters = _slamState.volumeSizeInMeters.x /
        (_dynamicOptions.highResMapping ? highResolutionVolumeBounds : lowResolutionVolumeBounds);
    
    // Avoid voxels that are too small - these become too noisy.
    voxelSizeInMeters = keepInRange(voxelSizeInMeters, 0.003, 0.2);
    
    // Compute the volume bounds in voxels, as a multiple of the volume resolution.
    GLKVector3 volumeBounds;
    volumeBounds.x = roundf(_slamState.volumeSizeInMeters.x / voxelSizeInMeters);
    volumeBounds.y = roundf(_slamState.volumeSizeInMeters.y / voxelSizeInMeters);
    volumeBounds.z = roundf(_slamState.volumeSizeInMeters.z / voxelSizeInMeters);
    
    NSLog(@"[Mapper] volumeSize (m): %f %f %f volumeBounds: %.0f %.0f %.0f (resolution=%f m)",
        _slamState.volumeSizeInMeters.x, _slamState.volumeSizeInMeters.y, _slamState.volumeSizeInMeters.z,
        volumeBounds.x, volumeBounds.y, volumeBounds.z,
        voxelSizeInMeters );
    
    NSDictionary* mapperOptions =
    @{
      kSTMapperLegacyKey: @(!_dynamicOptions.newMapperIsOn),
      kSTMapperVolumeResolutionKey: @(voxelSizeInMeters),
      kSTMapperVolumeBoundsKey: @[@(volumeBounds.x), @(volumeBounds.y), @(volumeBounds.z)],
      kSTMapperVolumeHasSupportPlaneKey: @(_slamState.cameraPoseInitializer.hasSupportPlane),
      kSTMapperEnableLiveWireFrameKey: @NO,
    };
    
    _slamState.mapper = [[STMapper alloc] initWithScene:_slamState.scene options:mapperOptions];
}

- (NSString*)maybeAddKeyframeWithDepthFrame:(STDepthFrame*)depthFrame
                                 colorFrame:(STColorFrame*)colorFrame
              depthCameraPoseBeforeTracking:(GLKMatrix4)depthCameraPoseBeforeTracking
{
    if (colorFrame == nil)
        return nil; // nothing to do

    // Only consider adding a new keyframe if the accuracy is high enough.
    if (_slamState.tracker.poseAccuracy < STTrackerPoseAccuracyApproximate)
        return nil;
    
    GLKMatrix4 depthCameraPoseAfterTracking = [_slamState.tracker lastFrameCameraPose];

    // Make sure the pose is in color camera coordinates in case we are not using registered depth.
    GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;
    [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m];
    GLKMatrix4 colorCameraPoseAfterTracking = GLKMatrix4Multiply(depthCameraPoseAfterTracking,
                                                                 colorCameraPoseInDepthCoordinateSpace);

    bool showHoldDeviceStill = false;
   
    // Check if the viewpoint has moved enough to add a new keyframe
    if ([_slamState.keyFrameManager wouldBeNewKeyframeWithColorCameraPose:colorCameraPoseAfterTracking])
    {
        const bool isFirstFrame = (_slamState.prevFrameTimeStamp < 0.);
        bool canAddKeyframe = false;
        
        if (isFirstFrame) // always add the first frame.
        {
            canAddKeyframe = true;
        }
        else // for others, check the speed.
        {
            float deltaAngularSpeedInDegreesPerSecond = FLT_MAX;
            NSTimeInterval deltaSeconds = depthFrame.timestamp - _slamState.prevFrameTimeStamp;
            
            // If deltaSeconds is 2x longer than the frame duration of the active video device, do not use it either
            CMTime frameDuration = self.videoDevice.activeVideoMaxFrameDuration;
            if (deltaSeconds < (float)frameDuration.value/frameDuration.timescale*2.f)
            {
                // Compute angular speed
                deltaAngularSpeedInDegreesPerSecond = deltaRotationAngleBetweenPosesInDegrees (depthCameraPoseBeforeTracking, depthCameraPoseAfterTracking)/deltaSeconds;
            }
            
            // If the camera moved too much since the last frame, we will likely end up
            // with motion blur and rolling shutter, especially in case of rotation. This
            // checks aims at not grabbing keyframes in that case.
            if (deltaAngularSpeedInDegreesPerSecond < _options.maxKeyframeRotationSpeedInDegreesPerSecond)
            {
                canAddKeyframe = true;
            }
        }
        
        if (canAddKeyframe)
        {
            [_slamState.keyFrameManager processKeyFrameCandidateWithColorCameraPose:colorCameraPoseAfterTracking
                                                                         colorFrame:colorFrame
                                                                         depthFrame:nil]; // Spare the depth frame memory, since we do not need it in keyframes.
        }
        else
        {
            // Moving too fast. Hint the user to slow down to capture a keyframe
            // without rolling shutter and motion blur.
            showHoldDeviceStill = YES;
        }
    }

    if (showHoldDeviceStill)
        return @"Please hold still so we can capture a keyframe...";
        
    return nil;
}

-(void) updateMeshAlphaForPoseAccuracy:(STTrackerPoseAccuracy)poseAccuracy
{
    switch (poseAccuracy)
    {
        case STTrackerPoseAccuracyHigh:
        case STTrackerPoseAccuracyApproximate:
            _display.meshRenderingAlpha = 0.8;
            break;
            
        case STTrackerPoseAccuracyLow:
        {
            _display.meshRenderingAlpha = 0.4;
            break;
        }
            
        case STTrackerPoseAccuracyVeryLow:
        case STTrackerPoseAccuracyNotAvailable:
        {
            _display.meshRenderingAlpha = 0.1;
            break;
        }
            
        default:
            NSLog(@"STTracker unknown pose accuracy.");
    };
}

- (void)processDepthFrame:(STDepthFrame *)depthFrame
          colorFrameOrNil:(STColorFrame*)colorFrame
{
    if (_options.applyExpensiveCorrectionToDepth)
    {
        NSAssert (!_options.useHardwareRegisteredDepth, @"Cannot enable both expensive depth correction and registered depth.");
        BOOL couldApplyCorrection = [depthFrame applyExpensiveCorrection];
        if (!couldApplyCorrection)
        {
            NSLog(@"Warning: could not improve depth map accuracy, is your firmware too old?");
        }
    }
    
    // Upload the new color image for next rendering.
    if (_useColorCamera && colorFrame != nil)
    {
        [self uploadGLColorTexture: colorFrame];
    }
    else if(!_useColorCamera)
    {
        [self uploadGLColorTextureFromDepth:depthFrame];
    }
    
    // Update the projection matrices since we updated the frames.
    {
        _display.depthCameraGLProjectionMatrix = [depthFrame glProjectionMatrix];
        if (colorFrame)
            _display.colorCameraGLProjectionMatrix = [colorFrame glProjectionMatrix];
    }
    
    switch (_slamState.scannerState)
    {
        case ScannerStateCubePlacement:
        {
            STDepthFrame* depthFrameForCubeInitialization = depthFrame;
            GLKMatrix4 depthCameraPoseInColorCoordinateFrame = GLKMatrix4Identity;
            
            // If we are using color images but not using registered depth, then use a registered
            // version to detect the cube, otherwise the cube won't be centered on the color image,
            // but on the depth image, and thus appear shifted.
            if (_useColorCamera && !_options.useHardwareRegisteredDepth)
            {
                GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;
                [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m];
                depthCameraPoseInColorCoordinateFrame = GLKMatrix4Invert(colorCameraPoseInDepthCoordinateSpace, nullptr);
                depthFrameForCubeInitialization = [depthFrame registeredToColorFrame:colorFrame];
            }
            
            // Provide the new depth frame to the cube renderer for ROI highlighting.
            [_display.cubeRenderer setDepthFrame:depthFrameForCubeInitialization];
            
            // Estimate the new scanning volume position.
            if (GLKVector3Length(_lastGravity) > 1e-5f)
            {
                bool success = [_slamState.cameraPoseInitializer updateCameraPoseWithGravity:_lastGravity depthFrame:depthFrameForCubeInitialization error:nil];
                
                // Since we potentially detected the cube in a registered depth frame, also save the pose
                // in the original depth sensor coordinate system since this is what we'll use for SLAM
                // to get the best accuracy.
                _slamState.initialDepthCameraPose = GLKMatrix4Multiply(_slamState.cameraPoseInitializer.cameraPose,
                                                                       depthCameraPoseInColorCoordinateFrame);
                
                NSAssert (success, @"Camera pose initializer error.");
            }
            
            // Tell the cube renderer whether there is a support plane or not.
            [_display.cubeRenderer setCubeHasSupportPlane:_slamState.cameraPoseInitializer.hasSupportPlane];
            
            // Enable the scan button if the pose initializer could estimate a pose.
            self.scanButton.enabled = _slamState.cameraPoseInitializer.hasValidPose;
            break;
        }
            
        case ScannerStateScanning:
        {
            // First try to estimate the 3D pose of the new frame.
            NSError* trackingError = nil;

            NSString* trackingMessage = nil;
            
            NSString* keyframeMessage = nil;
            
            GLKMatrix4 depthCameraPoseBeforeTracking = [_slamState.tracker lastFrameCameraPose];
            
            BOOL trackingOk = [_slamState.tracker updateCameraPoseWithDepthFrame:depthFrame colorFrame:colorFrame error:&trackingError];
            
            // Integrate it into the current mesh estimate if tracking was successful.
            if (!trackingOk)
            {
                NSLog(@"[Structure] STTracker Error: %@.", [trackingError localizedDescription]);

                trackingMessage = [trackingError localizedDescription];
            }
            else
            {
                // Update the tracking message.
                trackingMessage = computeTrackerMessage(_slamState.tracker.trackerHints);

                // Set the mesh transparency depending on the current accuracy.
                [self updateMeshAlphaForPoseAccuracy:_slamState.tracker.poseAccuracy];

                // If the tracker accuracy is high, use this frame for mapper update and maybe as a keyframe too.
                if (_slamState.tracker.poseAccuracy >= STTrackerPoseAccuracyHigh)
                {
                    [_slamState.mapper integrateDepthFrame:depthFrame cameraPose:[_slamState.tracker lastFrameCameraPose]];
                }
                
                keyframeMessage = [self maybeAddKeyframeWithDepthFrame:depthFrame colorFrame:colorFrame depthCameraPoseBeforeTracking:depthCameraPoseBeforeTracking];
                
                if (trackingMessage) // Tracking messages have higher priority.
                    [self showTrackingMessage:trackingMessage];
                else if (keyframeMessage)
                    [self showTrackingMessage:keyframeMessage];
                else
                    [self hideTrackingErrorMessage];
            }
            
            _slamState.prevFrameTimeStamp = depthFrame.timestamp;

            break;
        }
            
        case ScannerStateViewing:
        default:
        {} // Do nothing, the MeshViewController will take care of this.
    }
}

@end