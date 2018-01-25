/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import <MessageUI/MFMailComposeViewController.h>
#import <Structure/StructureSLAM.h>
#import "EAGLView.h"

@protocol MeshViewDelegate <NSObject>
- (void)meshViewWillDismiss;
- (void)meshViewDidDismiss;
- (BOOL)meshViewDidRequestColorizing:(STMesh*)mesh
            previewCompletionHandler:(void(^)(void))previewCompletionHandler
           enhancedCompletionHandler:(void(^)(void))enhancedCompletionHandler;
@end

@interface MeshViewController : UIViewController <UIGestureRecognizerDelegate, MFMailComposeViewControllerDelegate,UITextFieldDelegate>{
    
    

}

@property (nonatomic) NSString *nameCloud;

@property (nonatomic, assign) id<MeshViewDelegate> delegate;

@property (nonatomic) BOOL needsDisplay; // force the view to redraw.
@property (nonatomic) BOOL colorEnabled;
@property (nonatomic) STMesh * mesh;

@property (weak, nonatomic) IBOutlet UISegmentedControl *displayControl;
@property (weak, nonatomic) IBOutlet UILabel *meshViewerMessageLabel;
- (IBAction)savePointCloudButton:(id)sender;
- (IBAction)goOtherScreen:(id)sender;

- (IBAction)displayControlChanged:(id)sender;

- (IBAction)computeKeypointsButton:(id)sender;
@property (weak, nonatomic) IBOutlet UIButton *computeKeypointsOutlet;
- (IBAction)computeGlobalDescriptorButton:(id)sender;

//TODO For debug purpose
- (IBAction)debugPathButton:(id)sender;
@property (weak, nonatomic) IBOutlet UITextView *debugTextView;


- (void)showMeshViewerMessage:(NSString *)msg;
- (void)hideMeshViewerMessage;

- (void)setCameraProjectionMatrix:(GLKMatrix4)projRt;
- (void)resetMeshCenter:(GLKVector3)center;

- (void)setupGL:(EAGLContext*)context;

//C++ methods


@end
