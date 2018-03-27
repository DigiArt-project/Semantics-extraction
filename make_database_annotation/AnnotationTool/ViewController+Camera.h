/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#define HAS_LIBCXX

#import "ViewController.h"
#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#import "MeshViewController.h"

@interface ViewController (Camera) <AVCaptureVideoDataOutputSampleBufferDelegate>

- (void) startColorCamera;
- (void) stopColorCamera;
- (void) setColorCameraParametersForInit;
- (void) setColorCameraParametersForScanning;
- (void) setLensPositionWithValue:(float)value lockVideoDevice:(bool)lockVideoDevice;

- (bool) videoDeviceSupportsHighResColor;

@end
