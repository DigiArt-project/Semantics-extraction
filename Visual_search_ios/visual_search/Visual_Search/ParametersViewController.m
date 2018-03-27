//
//  ParametersViewController.m
//  Visual_Search
//
//  Created by Lirone Samoun on 19/01/2017.
//

//https://blog.engineyard.com/2013/developing-ios-push-notifications-nodejs

#import "ParametersViewController.h"
#include <AFNetworking/AFNetworking.h>
#include "AFHTTPSessionManager.h"

UIColor *colourRED, *colourBLUE, *colourGREEN, *colorBLACK;

@interface ParametersViewController (){
    
    UIActivityIndicatorView *m_spinner;
     MRProgressOverlayView *m_overlayIndicator;
    NSString *m_ratioTraining;
    NSString *m_type_descriptor;
    NSString *m_check_database;
    
}

@end

@implementation ParametersViewController


#pragma initialization
- (id)initWithNibName:(NSString *)nibNameOrNil
               bundle:(NSBundle *)nibBundleOrNil
{
    UIBarButtonItem *backButton = [[UIBarButtonItem alloc] initWithTitle:@"Back"
                                                                   style:UIBarButtonItemStylePlain
                                                                  target:self
                                                                  action:@selector(dismissView)];
    self.navigationItem.leftBarButtonItem = backButton;
    
    
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
        self.title = @"Parameters";
    }
    
    return self;
}


- (void)viewDidLoad {
    [super viewDidLoad];

    //Init color
    colourRED = [[UIColor alloc]initWithRed:255/255.0 green:102/255.0 blue:102.0/255.0 alpha:0.6];
    colourBLUE = [[UIColor alloc]initWithRed:0/255.0 green:51/255.0 blue:102.0/255.0 alpha:0.6];
    colourGREEN = [[UIColor alloc]initWithRed:102/255.0 green:204/255.0 blue:0/255.0 alpha:0.6];
    colorBLACK = [[UIColor alloc]initWithRed:0/255.0 green:0/255.0 blue:0/255.0 alpha:1];
    
    //Enable and disable certain button
    [self enableButton:_loadDatabaseButton];
    [self disableButton:_buildTreebutton];
    [self disableButton:_trainSVMButton];
    [self disableButton:_generateViewDatabaseButton];
    
    //init  the type dataset segmenter control
    if (self.typeOfDatasetControl.isOn){
        m_check_database = @"descriptors";
        _options_switch.objectsCheckDataset = NO;
        _options_switch.descriptorsCheck = YES;
    }else {
        m_check_database = @"dataset";
        _options_switch.objectsCheckDataset = YES;
        _options_switch.descriptorsCheck = NO;
    }
    
    //Init the spinner indicator
    m_spinner = [[UIActivityIndicatorView alloc] init];
    [m_spinner setCenter:CGPointMake(self.view.frame.size.width/2,self.view.frame.size.height/2)];
    m_spinner.activityIndicatorViewStyle = UIActivityIndicatorViewStyleWhiteLarge;
    m_spinner.color = colorBLACK;
    
    //Init the ratio training segmenter control
    if (_ratioTrainingControl.selectedSegmentIndex == 0){
        m_ratioTraining = @"2";
        
    }else if (_ratioTrainingControl.selectedSegmentIndex == 1){
        m_ratioTraining = @"3";
        
    }else if (_ratioTrainingControl.selectedSegmentIndex == 2){
        m_ratioTraining = @"5";
        
    }
    
    
    //Check connectivity
    [Utils checkBasicConnexion:^(BOOL internet)
     {
         if (internet)
         {
             //[self popupMessage:@"Connexion is okey" title:@"Ok"];
             // "Internet" aka Apple's region universal URL reachable
         }
         else
         {
             [self popupMessage:@"The Ipad is not connected to internet. Tasks won't work. Please check " title:@"Error"];
             // No "Internet" aka Apple's region universal URL un-reachable
         }
     }];

    
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)dismissView
{
    [self dismissViewControllerAnimated:YES completion:nil];
}



#pragma mark button

/*!
 @brief when the user click on this button, a call to the server is made and then a C++ program check if the dataset is well located and respect the condition
    If "objects" is selected, check if the dataset exists
    If "Descriptors" is selected, check if descriptors of the dataset have been computed
 
 Depending of the results, some button will be enabled or others disabled
 
 */
- (IBAction)onLoadDatabaseClicked:(id)sender {
    //Run the spinner
    [m_spinner startAnimating];
    [self.view addSubview:m_spinner];
    [NSThread detachNewThreadSelector:@selector(threadStartAnimating) toTarget:self withObject:nil];
    
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeIndeterminate;
    m_overlayIndicator.titleLabelText = @"Loading...";
    
    //Content type
    NSDictionary *params;
    params = @{@"enctype": @"multipart/form-data",
               @"checkDatabase": m_check_database,
               @"generateView": @"false",
               @"buildTree": @"false",
               @"trainSVMdescriptor": @"false"
               };

    
    //Create a HTTP request type POST for sending the data
    NSString *uploadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mParametersUrl];
    NSURLSessionConfiguration *configuration = [NSURLSessionConfiguration defaultSessionConfiguration];
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:configuration];
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    NSURLRequest *request = [[AFHTTPRequestSerializer serializer] requestWithMethod:@"POST" URLString:uploadHTTPLink parameters:params error:nil];
    
    
    NSURLSessionDataTask *dataTask = [manager dataTaskWithRequest:request completionHandler:^(NSURLResponse *response, id responseObject, NSError *error) {
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;

        //check the http response to see if the code has been well executed on the server side
        switch (httpResponse.statusCode) {
                /* Code in case of success == Server returned True  */
            case 200:
            {
                NSLog(@"Success");
                if (_options_switch.objectsCheckDataset){
                    NSString *str = @"[INFO] Dataset Okay ";
                    [self popupMessage:str title:@"Okay"];
                    [self writeToTextView:_debugLogTextView string:str erase:YES];
                    [self enableButton:_generateViewDatabaseButton];
                    [self enableButton:_buildTreebutton];
                    [self enableButton:_trainSVMButton];
                    self.viewLabel.backgroundColor = colourRED ;
                    self.descriptorLabel.backgroundColor = colourRED;
                    self.databaseLabel.backgroundColor = colourGREEN;
                }else {
                    NSString *str = @"[INFO] Descriptors of the database have been computed yet ";
                    [self popupMessage:str title:@"Okay"];
                    [self writeToTextView:_debugLogTextView string:str erase:YES];
                    [self disableButton:_generateViewDatabaseButton];
                    [self enableButton:_buildTreebutton];
                    [self enableButton:_trainSVMButton];
                    self.viewLabel.backgroundColor = colourGREEN ;
                    self.descriptorLabel.backgroundColor = colourGREEN;
                    self.databaseLabel.backgroundColor = colourGREEN;
                }
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Succeed";
                }
                [m_overlayIndicator dismiss:YES];

                break;
            }
                /* Code in case of error == Server returned False  */
            case 420:
            {
                NSLog(@"Error");
                if (_options_switch.objectsCheckDataset){
                    NSString *str = @"[ERROR] Dataset does not exist - Please check";
                    [self popupMessage:str title:@"ERROR"];
                    [self writeToTextView:_debugLogTextView string:str erase:YES];
                    [self disableButton:_generateViewDatabaseButton];
                    self.viewLabel.backgroundColor = colourRED ;
                    self.descriptorLabel.backgroundColor = colourRED;
                    self.databaseLabel.backgroundColor = colourRED;
                    
                }else {
                    NSString *str = @"[ERROR] Descriptors of the database have not been computed yet ";
                    [self popupMessage:str title:@"ERROR"];
                    [self writeToTextView:_debugLogTextView string:str erase:YES];
                    [self enableButton:_generateViewDatabaseButton];
                    self.viewLabel.backgroundColor = colourRED ;
                    self.descriptorLabel.backgroundColor = colourRED;
                    self.databaseLabel.backgroundColor = colourGREEN;
                }
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];

                break;
            }
            default:{
                /* Code in case of no success  == Server returned False */
                self.viewLabel.backgroundColor = colourRED ;
                self.descriptorLabel.backgroundColor = colourRED;
                self.databaseLabel.backgroundColor = colourRED;
                NSLog(@"Error");
                [m_spinner stopAnimating];
                break;
            }
        }
        
        
    }];
    [dataTask resume];
    
}

/*!
 @brief when the user click on this button, a call to the server is made and then a C++ program is launched. The code generate partial views and compute 3D descriptors of each objects of the dataset.
 
 */
- (IBAction)onGenerateViewsDatabaseClicked:(id)sender {
    
    //Start the spinner indicator
    [m_spinner startAnimating];
    [self.view addSubview:m_spinner];
    [NSThread detachNewThreadSelector:@selector(threadStartAnimating) toTarget:self withObject:nil];
    
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeIndeterminate;
    m_overlayIndicator.titleLabelText = @"Loading...";

    
    NSString *msg = @"[INFO] Generating views from the dataset and computing descriptors - Please wait a while... \n";
    [self writeToTextView:_debugLogTextView string:msg erase:YES];
    //Put the params for the HTTP request. Here everyhing is false exepted generateView
    NSDictionary *params = @{@"enctype": @"multipart/form-data",
                             @"checkDatabase": @"false",
                             @"generateView": @"true",
                             @"buildTree": @"false",
                             @"trainSVMdescriptor": @"false"
                             };
    
    
    
    //Create a HTTP request type POST for sending the data
    NSString *uploadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mParametersUrl];
    
    
    NSURLSessionConfiguration *configuration = [NSURLSessionConfiguration defaultSessionConfiguration];
    [configuration setTimeoutIntervalForRequest:100000];
    [configuration setTimeoutIntervalForResource:100000];
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:configuration];
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    NSURLRequest *request = [[AFHTTPRequestSerializer serializer] requestWithMethod:@"POST" URLString:uploadHTTPLink parameters:params error:nil];
    
    
    NSURLSessionDataTask *dataTask = [manager dataTaskWithRequest:request completionHandler:^(NSURLResponse *response, id responseObject, NSError *error) {
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;
        
        switch (httpResponse.statusCode) {
                /* Code in case of success == Server returned True  */
            case 200:
            {
                NSLog(@"Success");
                
                NSString *str = @"[INFO] Views have been generated and descriptors computed \n";
                [self popupMessage:str title:@"Okay"];
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self enableButton:_buildTreebutton];
                [self enableButton:_trainSVMButton];
                self.viewLabel.backgroundColor = colourGREEN ;
                self.descriptorLabel.backgroundColor = colourGREEN;
                self.databaseLabel.backgroundColor = colourGREEN;
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Succeed";
                }
                [m_overlayIndicator dismiss:YES];

                break;
            }
                /* Code in case of error == Server returned False  */
            case 420:
            {
                NSLog(@"Error");
                
                NSString *str = @"[ERROR] Problems during the process.. - Please check \n";
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self popupMessage:str title:@"ERROR"];

                [m_spinner stopAnimating];
                
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];
                break;
            }
            default:{
                NSLog(@"Error");
                [m_spinner stopAnimating];
                break;
            }
        }
        
        
    }];
    [dataTask resume];
    
    
    
}
/*!
 @brief when the user click on this button, a call to the server is made and then a C++ program is launched. The code build the index KDTree structure for similarity Search
 
 */
- (IBAction)onBuildTreeClicked:(id)sender {
    
    //Start the spinner indicator
    [m_spinner startAnimating];
    [self.view addSubview:m_spinner];
    [NSThread detachNewThreadSelector:@selector(threadStartAnimating) toTarget:self withObject:nil];
    
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeIndeterminate;
    m_overlayIndicator.titleLabelText = @"Loading...";

    
    NSString *msg = @"[INFO] Generating Index tree for similarity search - Please wait a while... \n";
    [self writeToTextView:_debugLogTextView string:msg erase:YES];

    //Parameters for HTTP request. Everything is set to false exept buildTree
    NSDictionary *params = @{@"enctype": @"multipart/form-data",
                             @"checkDatabase": @"false",
                             @"generateView": @"false",
                             @"buildTree": @"true",
                             @"trainSVMdescriptor": @"false"
                             };
    
    
    //Create a HTTP request type POST for sending the data
    NSString *uploadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mParametersUrl];
    
    
    NSURLSessionConfiguration *configuration = [NSURLSessionConfiguration defaultSessionConfiguration];
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:configuration];
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    NSURLRequest *request = [[AFHTTPRequestSerializer serializer] requestWithMethod:@"POST" URLString:uploadHTTPLink parameters:params error:nil];
    
    
    NSURLSessionDataTask *dataTask = [manager dataTaskWithRequest:request completionHandler:^(NSURLResponse *response, id responseObject, NSError *error) {
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;
        switch (httpResponse.statusCode) {
                /* Code in case of success == Server returned True  */
            case 200:
            {
                NSLog(@"Success");
                
                NSString *str = @"[INFO] Index tree for similarity search have been generated \n";
                [self popupMessage:str title:@"Okay"];
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self enableButton:_buildTreebutton];
                [self enableButton:_trainSVMButton];
                self.viewLabel.backgroundColor = colourGREEN ;
                self.descriptorLabel.backgroundColor = colourGREEN;
                self.databaseLabel.backgroundColor = colourGREEN;
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Succeed";
                }
                [m_overlayIndicator dismiss:YES];

                break;
            }
                 /* Code in case of error  == Server returned False  */
            case 420:
            {
                NSLog(@"Error");
                
                NSString *str = @"[ERROR] Problems during the process.. - Please check \n";
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self popupMessage:str title:@"ERROR"];
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];
                break;
            }
            default:{
                NSLog(@"Error");
                [m_spinner stopAnimating];
                break;
            }
        }
        
    }];
    [dataTask resume];

    
}
/*!
 @brief when the user click on this button, a call to the server is made and then a C++ program is launched. The code train an SVM with the chosen descriptor and the chosen ratio
 */
- (IBAction)onTrainSvmClicked:(id)sender {
    //Start the spinner indicator
    [m_spinner startAnimating];
    [self.view addSubview:m_spinner];
    [NSThread detachNewThreadSelector:@selector(threadStartAnimating) toTarget:self withObject:nil];
    
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeIndeterminate;
    m_overlayIndicator.titleLabelText = @"Loading...";

    NSString *msg = @"[INFO] Training SVM - Please wait a while... \n";
    [self writeToTextView:_debugLogTextView string:msg erase:YES];

    //Set the parameters for HTTP request
    NSDictionary *params;
    params = @{@"enctype": @"multipart/form-data",
                   @"checkDatabase": @"false",
                   @"trainSVMdescriptor": m_type_descriptor,
                   @"ratioTraining": m_ratioTraining
                   };
        
    
    
    //Create a HTTP request type POST for sending the data
    NSString *uploadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mParametersUrl];
    
    
    NSURLSessionConfiguration *configuration = [NSURLSessionConfiguration defaultSessionConfiguration];
    [configuration setTimeoutIntervalForRequest:100000];
    [configuration setTimeoutIntervalForResource:100000];
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:configuration];
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    
    NSURLRequest *request = [[AFHTTPRequestSerializer serializer] requestWithMethod:@"POST" URLString:uploadHTTPLink parameters:params error:nil];


    NSURLSessionDataTask *dataTask = [manager dataTaskWithRequest:request completionHandler:^(NSURLResponse *response, id responseObject, NSError *error) {
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;

        switch (httpResponse.statusCode) {
                /* Code in case of success == Server returned True  */
            case 200:
            {
                NSLog(@"Success");
                
                NSString *str = @"[INFO] SVM has been trained \n";
                [self popupMessage:str title:@"Okay"];
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self enableButton:_buildTreebutton];
                [self enableButton:_trainSVMButton];
                self.viewLabel.backgroundColor = colourGREEN ;
                self.descriptorLabel.backgroundColor = colourGREEN;
                self.databaseLabel.backgroundColor = colourGREEN;
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Succeed";
                }
                [m_overlayIndicator dismiss:YES];

                break;
            }
                /* Code in case of error == Server returned False  */

            case 420:
            {
                NSLog(@"Error");
                
                NSString *str = @"[ERROR] Problems during the SVM training.. - Please check \n";
                [self writeToTextView:_debugLogTextView string:str erase:NO];
                [self popupMessage:str title:@"ERROR"];
                [m_spinner stopAnimating];
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];
                break;
            }
            default:{
                break;
            }
        }
        
    }];
    [dataTask resume];

}


#pragma utilities method
/*!
 @brief Display a pop up message
 
 @param msg the message we want to display
 titl the title of the message
 
 */
-(void)popupMessage:(NSString*)msg title:(NSString*)titl{
    UIAlertController* alert = [UIAlertController alertControllerWithTitle:titl
                                                                   message:msg
                                                            preferredStyle:UIAlertControllerStyleAlert];
    //After the user click on okey, convertion from obj to pcd
    UIAlertAction* defaultAction = [UIAlertAction actionWithTitle:@"OK"
                                                            style:UIAlertActionStyleDefault
                                                          handler:^(UIAlertAction * action) {
                                                          }];
    [alert addAction:defaultAction];
    [self presentViewController:alert animated:YES completion:nil];
    
}

/*!
 @brief enable a button and change the colour background to blue
 
 @param button the button we want to affect

 */
-(void)enableButton:(UIButton*)button {
    button.hidden = NO;
    button.backgroundColor = colourBLUE;
    button.enabled = YES;
}
/*!
 @brief disable a button and change the colour background to red
 
 @param button the button we want to affect
 
 */
-(void)disableButton:(UIButton*)button {
    button.backgroundColor = colourRED;
    button.enabled = NO;
}




/*!
 @brief write a message in a given textView field
 
 @param textView the textView Field where we want to write content
 del if we want to remove all the previous content on the textView field or we want to append the data
 
 */
-(void)writeToTextView:(UITextView*)textView string:(NSString*)str erase:(BOOL)del{
    if (del){
        textView.text = str;
    }else {
        NSString *tmp = textView.text;
        textView.text = [NSString stringWithFormat:@"%@ %@", tmp, str];
    }
}

- (void) threadStartAnimating {
    [m_spinner startAnimating];
}

#pragma kvo
- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context
{
    //[super observeValueForKeyPath:keyPath ofObject:object change:change context:context];
    
    if ([keyPath isEqualToString:@"fractionCompleted"] && [object isKindOfClass:[NSProgress class]]) {
        NSProgress *progress = (NSProgress *)object;
        NSString *info_str = [NSString stringWithFormat: @"[IOS INFO] - Progress : %f \n",progress.fractionCompleted*100];
        //self.debugLogTextField.text = [self.debugLogTextField.text stringByAppendingString:info_str];
        //NSLog(@"Progress :  %f", progress.fractionCompleted*100);
    }
    if ([object isFinished]) {
        @try {
            [object removeObserver:self
                        forKeyPath:@"fractionCompleted"
                           context:NULL];
            
        }
        @catch (NSException * __unused exception) {}
    }
}

- (void)unregisterAsObserverForObject:(NSProgress*)progress {
    [progress removeObserver:self
                  forKeyPath:@"fractionCompleted"
                     context:NULL];
    
}
- (IBAction)descriptorsTrainingSwitched:(id)sender {
    if (_descriptorsTrainingControl.selectedSegmentIndex == 0){
        m_type_descriptor = @"esf";
        
    }else if (_descriptorsTrainingControl.selectedSegmentIndex == 1){
        m_type_descriptor = @"vfh";
        
    }else if (_descriptorsTrainingControl.selectedSegmentIndex == 2){
        m_type_descriptor = @"cvfh";
        
    }else if (_descriptorsTrainingControl.selectedSegmentIndex == 3){
        m_type_descriptor = @"ourcvfh";
        
    }

}
- (IBAction)ratioTrainingSwitched:(id)sender {
    if (_ratioTrainingControl.selectedSegmentIndex == 0){
        m_ratioTraining = @"2";
        
    }else if (_ratioTrainingControl.selectedSegmentIndex == 1){
        m_ratioTraining = @"3";
        
    }else if (_ratioTrainingControl.selectedSegmentIndex == 2){
        m_ratioTraining = @"5";
        
    }
}


- (IBAction)onTypeOfDatasetSwitched:(id)sender {
    if (self.typeOfDatasetControl.isOn){
        m_check_database = @"descriptors";
        _options_switch.descriptorsCheck = YES;
        _options_switch.objectsCheckDataset = NO;
    }else {
        m_check_database = @"dataset";
        _options_switch.objectsCheckDataset = YES;
        _options_switch.descriptorsCheck = NO;
    }
}
@end
