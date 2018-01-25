//
//  SearchViewController.h
//  Visual_Search
//
//  Created by Lirone Samoun on 06/10/2016.
//

/*!
 @class SearchViewController
 
 @brief The searchViewController class - Part where the user perform the retrieval search

 */

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import <MessageUI/MFMailComposeViewController.h>
#import <Structure/StructureSLAM.h>
#import "Category.h"
#import "EAGLView.h"
#import "ParametersViewController.h"


@interface SearchViewController : UIViewController {
    
    //Parameters view controllers
    UINavigationController *_parametersViewNavigationController;
    ParametersViewController *_parametersViewController;
}

//Cloud name got from the previous view controller
@property (nonatomic) NSString *m_cloud_name;


//Button action
- (IBAction)onUploadPointCloudClick:(id)sender;
- (IBAction)onParametersClick:(id)sender;
- (IBAction)onGetResultsClick:(id)sender;
@property (weak, nonatomic) IBOutlet UIButton *getResultsButton;
- (IBAction)onConnexionClick:(id)sender;
@property (weak, nonatomic) IBOutlet UIButton *connexionButton;

//Segmenter control
- (IBAction)methodClassificationOnSwitch:(id)sender;
- (IBAction)descriptorsClassificationOnSwitch:(id)sender;
@property (weak, nonatomic) IBOutlet UISegmentedControl *methodClassificationControl;
@property (weak, nonatomic) IBOutlet UISegmentedControl *descriptorsClassificationControl;



//Label and text view
/*! @brief The debug log textfield to display debug message */
@property (weak, nonatomic) IBOutlet UITextView *debugLogTextField;
/*! @brief Image from the result category predicted by SVM. */
@property (weak, nonatomic) IBOutlet UIImageView *resultImageCategoryImageView;
/*! @brief Label which shows the method chose */
@property (weak, nonatomic) IBOutlet UILabel *methodChoseLabel;
/*! @brief Label which shows the descriptor chose */
@property (weak, nonatomic) IBOutlet UILabel *descriptorChoseLabel;
/*! @brief The table which shows all the results from the similarity search */
@property (weak, nonatomic) IBOutlet UITableView *classificationResultTableView;
/*! @brief Label which shows the category predicted from the SVM */
@property (weak, nonatomic) IBOutlet UILabel *categoryResultSvmLabel;

@end
