//
//  ParametersViewController.h
//  Visual_Search
//
//  Created by Lirone Samoun on 19/01/2017.

#import <UIKit/UIKit.h>
#import <MRProgress/MRProgress.h>
#import "Utils.h"

// See default initialization in: -(void)initializeDynamicOptions
struct Options_switch
{
    /*! @brief bool indicating we are going to check the existence or not of the dataset */
    bool objectsCheckDataset;
    /*! @brief bool indicating we are going to check or not the descriptors on the dataset */
    bool descriptorsCheck;
};


@interface ParametersViewController : UIViewController{
    
    struct Options_switch _options_switch;
    
}

//Buttons
- (IBAction)onLoadDatabaseClicked:(id)sender;
- (IBAction)onGenerateViewsDatabaseClicked:(id)sender;
- (IBAction)onBuildTreeClicked:(id)sender;
- (IBAction)onTrainSvmClicked:(id)sender;
@property (weak, nonatomic) IBOutlet UIButton *generateViewDatabaseButton;
@property (weak, nonatomic) IBOutlet UIButton *buildTreebutton;
@property (weak, nonatomic) IBOutlet UIButton *trainSVMButton;
@property (weak, nonatomic) IBOutlet UIButton *loadDatabaseButton;


//TextView
@property (weak, nonatomic) IBOutlet UITextView *debugLogTextView;

//Switch
- (IBAction)onTypeOfDatasetSwitched:(id)sender;
@property (weak, nonatomic) IBOutlet UISwitch *typeOfDatasetControl;

//Segmenter control
- (IBAction)descriptorsTrainingSwitched:(id)sender;
@property (weak, nonatomic) IBOutlet UISegmentedControl *descriptorsTrainingControl;
- (IBAction)ratioTrainingSwitched:(id)sender;
@property (weak, nonatomic) IBOutlet UISegmentedControl *ratioTrainingControl;

//Label
@property (weak, nonatomic) IBOutlet UILabel *databaseLabel;
@property (weak, nonatomic) IBOutlet UILabel *descriptorLabel;
@property (weak, nonatomic) IBOutlet UILabel *viewLabel;


@end
