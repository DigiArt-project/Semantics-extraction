//
//  SearchViewController.m
//  Visual_Search
//
//  Created by Lirone Samoun on 06/10/2016.
//

#import <Foundation/Foundation.h>
#import <MRProgress/MRProgress.h>
#include <AFNetworking/AFNetworking.h>
#include <AFNetworking/AFHTTPSessionManager.h>
#import <SystemConfiguration/SystemConfiguration.h>
#import "SearchViewController.h"
#import "SimpleTableCell.h"
#import "Utils.h"

BOOL svm_esf_available = YES;
BOOL svm_cvfh_available = NO;
BOOL svm_vfh_available = NO;
BOOL svm_ourcvfh_available = NO;


@interface SearchViewController() <UIGestureRecognizerDelegate,UITableViewDataSource,UITableViewDelegate,NSURLSessionDataDelegate, NSURLSessionDelegate, NSURLSessionTaskDelegate>
{
    UIActivityIndicatorView *m_spinner;
    MRProgressOverlayView *m_overlayIndicator;
    //Array which contains all the existing categories
    NSMutableArray *categories_arr;
    //Array for the table views
    NSMutableArray *result_cat_array;
    NSMutableArray *thumbnails_array;
    NSMutableArray *number_array;
    NSMutableArray *distance_array;
    
    NSString *m_method_search_selected, *m_type_descriptor_selected, *m_type_dataset_selected;
    
    BOOL m_isConnectedToInternet, m_isConnectedToServer;
    
}

@end

@implementation SearchViewController

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
        self.title = @"Search view";
    }
    
    return self;
}

/*!
 @brief Load categories from a plist file
 Our plist file contains all the categories references on the database
 
 Remark : don't use now this function in the code
 
 @param name_file the name of the plist file
 
 */
-(NSMutableArray*) loadCategoriesFromPlistFile:(NSString*)name_file{
    // Find out the path of recipes.plist
    NSString *path = [[NSBundle mainBundle] pathForResource:name_file ofType:@"plist"];
    
    // Load the file content and read the data into arrays
    NSDictionary *dict = [[NSDictionary alloc] initWithContentsOfFile:path];
    //There are three keys (name, Thumbnail, description) in the property list. In the example, each key is associated with a specific array, which is the value
    //These lines of code retrieves the corresponding array with the key we defined earlier.
    NSArray *tableData = [dict objectForKey:@"categoryName"];
    NSArray *thumbnails = [dict objectForKey:@"thumbnail"];
    NSArray *description = [dict objectForKey:@"description"];
}

/*!
 @brief Load categories from a json file
 Our json file contains all the categories references on the database
 
 @param name_file the name of the plist file
 
 */
-(NSMutableArray*) loadCategoriesFromJasonFileCategories:(NSString*)name_file{
    NSMutableArray *categories = [NSMutableArray array];
    //Get the file from our bundle ressource
    NSString *filePathCategories = [[NSBundle mainBundle] pathForResource:name_file ofType:@"json"];
    if (!filePathCategories) {
        NSLog(@"Error - file path not found");
    }else {
        //Transform the data to NSData for sending to the server
        NSData *objectData = [NSData dataWithContentsOfFile:filePathCategories];
        NSError *jsonError;
        NSArray *jsonDictionnary = [NSJSONSerialization JSONObjectWithData:objectData
                                                                   options:NSJSONReadingAllowFragments
                                                                     error:&jsonError];
        //for checking if the received json is an array or dictionary:
        if(jsonError != nil){
            NSLog(@"[ERROR] JSON Error: %@", jsonError);
            exit(1);
        }
        int count = 0;
        for (NSDictionary *item in jsonDictionnary) {
            count ++;
            //Get the key from the json file
            NSString *name = item[@"name"];
            NSString *url_cover = item[@"image"];
            UIImage* m_cover  = [UIImage imageNamed:url_cover];
            NSString *description = item[@"description"];
            
            [categories addObject:[[Category alloc] initWithName:name description:description cover:m_cover]];
            
        }
    }
    return categories;
    
}


- (void)viewDidLoad
{
    [super viewDidLoad];
    
    
    self.getResultsButton.hidden = YES;
    
    //Segmenter method classification control
    if (self.methodClassificationControl.selectedSegmentIndex == 0){
        //SVM method
        m_method_search_selected = @"svm";
        self.methodChoseLabel.text = @"SVM";
        self.resultImageCategoryImageView.hidden = NO;
        self.categoryResultSvmLabel.hidden = NO;
        self.classificationResultTableView.hidden = YES;
    }
    else {
        //Similarity Search
        m_method_search_selected = @"similarity";
        self.methodChoseLabel.text = @"Similarity Search";
        self.resultImageCategoryImageView.hidden = YES;
        self.categoryResultSvmLabel.hidden = YES;
        self.classificationResultTableView.hidden = NO;
        
    }
    //Segmenter descriptor control
    if (self.descriptorsClassificationControl.selectedSegmentIndex == 0){
        m_type_descriptor_selected = @"esf";
        self.descriptorChoseLabel.text = @"ESF";
    }else if (self.descriptorsClassificationControl.selectedSegmentIndex == 1){
        m_type_descriptor_selected = @"cvfh";
        self.descriptorChoseLabel.text = @"CVFH";
    }
    else if (self.descriptorsClassificationControl.selectedSegmentIndex == 3){
        m_type_descriptor_selected = @"gshot";
        self.descriptorChoseLabel.text = @"GSHOT";
    }
    else {
        m_type_descriptor_selected = @"vfh";
        self.descriptorChoseLabel.text = @"VFH";
    }
    
    if (self.datasetClassificationControl.selectedSegmentIndex == 0){
        m_type_dataset_selected = @"structure";
        self.methodChoseLabel.text = @"Structure Sensor";
        categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_structure"];
    }else if (self.datasetClassificationControl.selectedSegmentIndex == 1){
        m_type_dataset_selected = @"pottery";
        self.methodChoseLabel.text = @"Pottery";
        categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_pottery"];
    }
    else if (self.datasetClassificationControl.selectedSegmentIndex == 2){
        m_type_dataset_selected = @"Pottery Mix";
        self.methodChoseLabel.text = @"Potter Mix";
        categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_potterymix"];
    }
    

    //We get from our json file categories.json the categories available for our retrieval process
    //categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories"];
    
    NSString *str = [NSString stringWithFormat:@"%s %lu", "Categories available : ",  (unsigned long)[categories_arr count]];
    [self writeToTextView:self.debugLogTextField string:str erase:YES];
    
    //Init array
    result_cat_array = [NSMutableArray array];
    thumbnails_array =  [NSMutableArray array];
    number_array =  [NSMutableArray array];
    distance_array =  [NSMutableArray array];
    
    // Remove table cell separator
    [self.classificationResultTableView setSeparatorStyle:UITableViewCellSeparatorStyleNone];
    
    // Assign our own backgroud for the view
    self.parentViewController.view.backgroundColor = [UIColor colorWithPatternImage:[UIImage imageNamed:@"images/common_bg"]];
    self.classificationResultTableView.backgroundColor = [UIColor clearColor];
    
    // Add padding to the top of the table view
    UIEdgeInsets inset = UIEdgeInsetsMake(5, 0, 0, 0);
    self.classificationResultTableView.contentInset = inset;
    
    m_spinner = [[UIActivityIndicatorView alloc] init];
    [m_spinner setCenter:CGPointMake(self.view.frame.size.width/2,self.view.frame.size.height/2)];
    m_spinner.activityIndicatorViewStyle = UIActivityIndicatorViewStyleWhiteLarge;
    
    [self setupParametersViewController];
    
    
    [Utils checkBasicConnexion:^(BOOL internet)
     {
         
         if (internet)
         {
             m_isConnectedToInternet = YES;
             [Utils checkServerConnection:^(BOOL server)
              {
                  if (server)
                  {
                      m_isConnectedToServer = YES;
                      [self popupMessage:@"Connexion is okey" title:@"Ok"];
                      self.connexionButton.backgroundColor = [Utils colorGREEN];
                      [self.connexionButton setTitle:@"Connexion OK" forState:UIControlStateNormal];
                      m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                      if (m_overlayIndicator.titleLabel.text.length > 0) {
                          m_overlayIndicator.titleLabelText = @"Connexion OK";
                      }
                      [m_overlayIndicator dismiss:YES];
                      
                  }
                  else
                  {
                      m_isConnectedToServer = NO;
                      [self popupMessage:@"Basic connexion OK - Connexion server : failed" title:@"Error"];
                      self.connexionButton.backgroundColor = [Utils colorRED];
                      [self.connexionButton setTitle:@"Connexion failed" forState:UIControlStateNormal];
                      m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                      if (m_overlayIndicator.titleLabel.text.length > 0) {
                          m_overlayIndicator.titleLabelText = @"Connexion Failed";
                      }
                      [m_overlayIndicator dismiss:YES];
                  }
              }];
         }
         else
         {
             m_isConnectedToInternet = NO;
             [self popupMessage:@"No connexion " title:@"Error"];
             self.connexionButton.backgroundColor = [Utils colorRED];
             [self.connexionButton setTitle:@"Connexion failed" forState:UIControlStateNormal];
             m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
             if (m_overlayIndicator.titleLabel.text.length > 0) {
                 m_overlayIndicator.titleLabelText = @"Connexion Failed";
             }
             [m_overlayIndicator dismiss:YES];
             
         }
     }];
    
    
}


- (void)dismissView
{
    [self dismissViewControllerAnimated:YES completion:nil];
}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}


-(void)setupParametersViewController
{
    
    _parametersViewController = [[ParametersViewController alloc] initWithNibName:@"ParametersViewController" bundle:nil];
    _parametersViewNavigationController = [[UINavigationController alloc] initWithRootViewController:_parametersViewController];
    
}

- (void)presentParametersViewcontroller
{
    
    [self presentViewController:_parametersViewNavigationController animated:YES completion:^{}];
    
}


- (void)dealloc
{
    
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

#pragma mark - Documents manager function

/*!
 @brief display all the file inside the document directory of the device
 */
-(void)displayDirectoryContent{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSError * error;
    NSArray * directoryContents =  [[NSFileManager defaultManager]
                                    contentsOfDirectoryAtPath:documentsDirectory error:&error];
    
    NSLog(@"directoryContents ====== %@",directoryContents);
}

/*!
 @brief Function which checks if a file exists in the document directory of the device
 */
-(bool)checkIfExist:(NSString*)filename{
    NSString* path =  [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory,    NSUserDomainMask, YES) objectAtIndex:0];
    path = [path stringByAppendingPathComponent:filename];
    BOOL fileExists = [[NSFileManager defaultManager] fileExistsAtPath:path];
    if (fileExists) {
        NSLog(@"file exists");
        return true;
    }
    else
    {
        return false;
    }
}

/*!
 @brief Remove a specific file from the document directory of the device
 
 @param filename the file we want to delete from the document directory
 
 */
- (void)removeDataFromDocumentDirectory:(NSString *)filename
{
    NSFileManager *fileManager = [NSFileManager defaultManager];
    NSString *documentsPath = [NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES) objectAtIndex:0];
    
    NSString *filePath = [documentsPath stringByAppendingPathComponent:filename];
    NSError *error;
    BOOL success = [fileManager removeItemAtPath:filePath error:&error];
    if (success) {
        NSLog(@"Ok file deleted");
    }
    else
    {
        NSLog(@"Could not delete file -:%@ ",[error localizedDescription]);
    }
}


/*!
 @brief Check if a string contains another string
 */
-(bool)IsString:(NSString *)str contains:(NSString*)substr
{
    NSRange range = [str  rangeOfString: substr options: NSCaseInsensitiveSearch];
    //NSLog(@"found: %@", (range.location != NSNotFound) ? @"Yes" : @"No");
    if (range.location != NSNotFound) {
        return true;
    }else {
        return false;
    }
}

#pragma mark - tableview
- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    return 1;
}

/*!
 @brief creates and returns a cell with the name of the category, the distance and the cover image
 */
- (UITableViewCell*)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    static NSString *simpleTableIdentifier = @"SimpleTableCell";
    
    SimpleTableCell *cell = (SimpleTableCell *)[tableView dequeueReusableCellWithIdentifier:simpleTableIdentifier];
    if (cell == nil)
    {
        NSArray *nib = [[NSBundle mainBundle] loadNibNamed:@"SimpleTableCell" owner:self options:nil];
        cell = [nib objectAtIndex:0];
    }
    
    /*NSLog(@"Category Label size :  %lu", (unsigned long)[result_cat_array count]);
     NSLog(@"Thumbmail image size :  %lu", (unsigned long)[thumbnails_array count]);
     NSLog(@"distance Label size :  %lu", (unsigned long)[distance_array count]);
     NSLog(@"number array size :  %lu", (unsigned long)[number_array count]);*/
    
    cell.categoryLabel.text = [result_cat_array objectAtIndex:indexPath.row];
    cell.thumbnailImageView.image = [thumbnails_array objectAtIndex:indexPath.row];
    cell.distanceLabel.text = [distance_array objectAtIndex:indexPath.row];
    cell.numberLabel.text = [number_array objectAtIndex:indexPath.row];
    
    // Assign our own background image for the cell
    UIImage *background = [self cellBackgroundForRowAtIndexPath:indexPath];
    
    UIImageView *cellBackgroundView = [[UIImageView alloc] initWithImage:background];
    cellBackgroundView.image = background;
    cell.backgroundView = cellBackgroundView;
    
    return cell;
    
}

/*!
 @brief returns the number of rows to display in the table view, which matches the number of categories in the data structure.
 */
- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    return [result_cat_array count];
}

/*!
 @brief apply a specific background for our cell in our tableView
 */
- (UIImage *)cellBackgroundForRowAtIndexPath:(NSIndexPath *)indexPath
{
    NSInteger rowCount = [self tableView:[self classificationResultTableView] numberOfRowsInSection:0];
    NSInteger rowIndex = indexPath.row;
    UIImage *background = nil;
    
    if (rowIndex == 0) {
        background = [UIImage imageNamed:@"images/cell_top.png"];
    } else if (rowIndex == rowCount - 1) {
        background = [UIImage imageNamed:@"images/cell_bottom.png"];
    } else {
        background = [UIImage imageNamed:@"image/cell_middle.png"];
    }
    
    return background;
}


#pragma mark - results

/*!
 @brief Download the results from the server after having performed similarity search or svm. Then parse the results and display.
 */
-(void)downloadResultFromServer{
    
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeDeterminateCircular;
    m_overlayIndicator.titleLabelText = @"Getting results...";
    
    NSString* file;
    if (self.methodClassificationControl.selectedSegmentIndex == 0){
        //SVM
        [self writeToTextView:_debugLogTextField string:@"Results SVM :  \n" erase:YES];
        file =  [self.m_cloud_name stringByAppendingString:@"_result.label"];
        if ([self checkIfExist:file]){
            [self removeDataFromDocumentDirectory:file];
        }
    }else {
        //Similarity search
        [self writeToTextView:_debugLogTextField string:@"Results Similarity Search :  \n" erase:YES];
        //Check if results file already in the folder to delete it in order to download it again (otherwise the file is not ecrased)
        file =  [self.m_cloud_name stringByAppendingString:@"_result.json"];
        if ([self checkIfExist:file]){
            [self removeDataFromDocumentDirectory:file];
        }
        
    }
    
    // enable the network activity indicator on the status bar to indicate to the user that a network process is running.
    [[UIApplication sharedApplication] setNetworkActivityIndicatorVisible:YES];
    
    //Create a http request for downloading the json file
    NSURLSessionConfiguration *configuration = [NSURLSessionConfiguration defaultSessionConfiguration];
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:configuration];
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    NSString *downloadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mDownload];
    NSURL *URL = [NSURL URLWithString:downloadHTTPLink];
    NSURLRequest *request = [NSURLRequest requestWithURL:URL];
    
    NSURLSessionDownloadTask *downloadTask = [manager downloadTaskWithRequest:request progress:nil destination:^NSURL *(NSURL *targetPath, NSURLResponse *response) {
        
        dispatch_async(dispatch_get_main_queue(), ^{
            [[UIApplication sharedApplication] setNetworkActivityIndicatorVisible:NO];
        });
        
        NSError *error;
        NSURL *documentsDirectoryURL = [[NSFileManager defaultManager] URLForDirectory:NSDocumentDirectory inDomain:NSUserDomainMask appropriateForURL:nil create:NO error:&error];
        if (error) {
            NSString *str = [NSString stringWithFormat: @"[IOS DOWNLOAD] - ERROR message : %@ \n", error];
            [self writeToTextView:_debugLogTextField string:str erase:NO];
            [self popupMessage:@"Error during the download step" title:@"Error"];
            
        }
        return [documentsDirectoryURL URLByAppendingPathComponent:[response suggestedFilename]];
    } completionHandler:^(NSURLResponse *response, NSURL *filePath, NSError *error) {
        if (error) {
            NSString *str = [NSString stringWithFormat: @"[IOS ERROR] - ERROR Download step : %@ \n", error];
            [self popupMessage:str title:@"Error"];
            
        } else {
            //SVM
            if (self.methodClassificationControl.selectedSegmentIndex == 0){
                NSLog(@"go parsing SVM");
                [self parseResultsSVMandDisplay];
                
            }
            //Similarity search
            else {
                //Parse the json file
                NSLog(@"go parsing SVM");
                [self parseResultsSimilaritySearchandDisplay];
            }
            m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
            if (m_overlayIndicator.titleLabel.text.length > 0) {
                m_overlayIndicator.titleLabelText = @"Succeed";
            }
            [m_overlayIndicator dismiss:YES];
        }
    }];
    [downloadTask resume];
    
}

/*!
 @brief Parse the json file which contains the result label and category of the prediction. Display then the result to the imageView
 */
-(void) parseResultsSVMandDisplay {
    
    self.resultImageCategoryImageView.image = nil;
    
    NSString *documentsDirectoryPath = [NSSearchPathForDirectoriesInDomains( NSDocumentDirectory, NSUserDomainMask, YES ) objectAtIndex:0];
    NSString *result_label = [self.m_cloud_name stringByAppendingString:@"_result.label"];
    // get a reference to our file
    NSString *result_label_path = [documentsDirectoryPath stringByAppendingPathComponent:result_label];
    //Check if the file exist
    BOOL fileExists = [[NSFileManager defaultManager] fileExistsAtPath:result_label_path];
    if (!fileExists){
        [self popupMessage:@"Result file SVM does not exist. Please check if .label file exists " title:@"Error"];
        return;
        
    }
    
    // read the contents into a string
    NSString *contentResult = [[NSString alloc]initWithContentsOfFile:result_label_path encoding:NSUTF8StringEncoding error:nil];
    // split our string into an array
    NSArray *contentPlit = [contentResult componentsSeparatedByString:@"\n"];
    
    //First position Corresponds to label
    NSString *label = [contentPlit objectAtIndex:0];
    NSLog(@"Label:  %@", label);
    //Second position Corresponds to category
    NSString *cat_found = [contentPlit objectAtIndex:1];
    NSLog(@"Category:  %@", cat_found);
    
    NSString *str = [NSString stringWithFormat:@"Label : %@\nCategory : %@ \n", label, cat_found];
    [self writeToTextView:_debugLogTextField string:str erase:YES];
    
    
    BOOL isFound = NO;
    for (Category * category in categories_arr) {
        if ([self IsString:[category m_name] contains:cat_found]) {
            _resultImageCategoryImageView.image = [category m_cover];
            NSLog(@"Image content : %@ ", [category m_cover]);
            self.categoryResultSvmLabel.text = [category m_name];
            isFound = YES;
        }
    }
    if (!isFound){
        self.categoryResultSvmLabel.text = @"Unknown category";
    }
}

/*!
 @brief Parse the json file which contains the result of similarity search. Display then the result to the tableView
 */
-(void) parseResultsSimilaritySearchandDisplay{
    NSLog(@"[INFO] Parsing JSON file for similarity search");
    //Remove the results if we want to display new results
    [result_cat_array removeAllObjects];
    [distance_array removeAllObjects];
    [number_array removeAllObjects];
    [thumbnails_array removeAllObjects];
    //Remove the results if we want to display new results
    //[self.classificationResultTableView reloadData];
    
    NSString *documentsDirectoryPath = [NSSearchPathForDirectoriesInDomains( NSDocumentDirectory, NSUserDomainMask, YES ) objectAtIndex:0];
    NSString *result_jsonFile = [self.m_cloud_name stringByAppendingString:@"_result.json"];
    NSString *result_jsonFile_path = [documentsDirectoryPath stringByAppendingPathComponent:result_jsonFile];
    //Check if the file exist
    BOOL fileExists = [[NSFileManager defaultManager] fileExistsAtPath:result_jsonFile_path];
    if (!fileExists){
        [self popupMessage:@"Result Similarity Search file does not exist. Please check if .json file exists. " title:@"Error"];
        return;
    }
    
    //Transform the data to NSData for sending to the server
    NSData *objectData = [NSData dataWithContentsOfFile:result_jsonFile_path];
    NSError *jsonError;
    NSArray *jsonDictionnary = [NSJSONSerialization JSONObjectWithData:objectData
                                                               options:NSJSONReadingAllowFragments
                                                                 error:&jsonError];
    //for checking if the received json is an array or dictionary:
    if(jsonError != nil){
        NSLog(@"[ERROR] JSON Error: %@", jsonError);
        return;
    }
    int count = 0;
    for (NSDictionary *item in jsonDictionnary) {
        
        NSString *current_category = item[@"Category"];
        NSString *current_number = item[@"Number"];
        NSString *current_dist = item[@"Distance"];
        NSString *str = [NSString stringWithFormat: @" \n N°- %@ - Category : %@ \n distance = %@", current_number, current_category, current_dist];
        NSLog(@"[INFO] str date %@ :", str);
        
        [self writeToTextView:_debugLogTextField string:str erase:NO];
        
        for (Category * category in categories_arr) {
             //NSLog(@"Category ARRAY : %@",[category m_name]);
            //[self writeToTextView:_debugLogTextField string:[category m_name] erase:NO];
            if ([self IsString:current_category contains:[category m_name]]) {
                count ++;
                [result_cat_array addObject:[category m_name]];
                [thumbnails_array addObject:[category m_cover]];
                [number_array addObject:[NSString stringWithFormat:@"%@", current_number]];
                [distance_array addObject:[NSString stringWithFormat:@"%@", current_dist]];
                NSLog(@"Category found : %@ with distance %@ ",[category m_name],current_dist );
                [self.classificationResultTableView reloadData];
            }
        }
    }
    
}

#pragma mark - buttons action
/*!
 @brief when the user click on upload button
 */
- (IBAction)onUploadPointCloudClick:(id)sender {
    
    if (!m_isConnectedToServer){
        [self popupMessage:@"You are not connected to the server - Abort" title:@"Error"];
        return;
    }
    
    //Upload indicator spinn activity
    [m_spinner startAnimating];
    [self.view addSubview:m_spinner];
    [NSThread detachNewThreadSelector:@selector(threadStartAnimating) toTarget:self withObject:nil];
    
    // Init the progress overlay as usual
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeDeterminateCircular;
    m_overlayIndicator.titleLabelText = @"Uploading...";
    
    [self writeToTextView:self.debugLogTextField string:@"[INFO] Upload. Please wait...\n " erase:YES];
    
    
    //Get the path to the documents directory of the device
    NSString *documentsDirectoryPath = [NSSearchPathForDirectoriesInDomains( NSDocumentDirectory, NSUserDomainMask, YES ) objectAtIndex:0];
    
    //Content type
    NSDictionary *params;
    params = @{@"enctype": @"multipart/form-data",
               @"type_descriptor_selected": m_type_descriptor_selected,
               @"method_search_selected": m_method_search_selected,
               @"type_dataset_selected": m_type_dataset_selected,
               };
    
    NSString *cloud_obj_path;
    //Save the point cloud to the device
    NSString *cloud_obj = [self.m_cloud_name stringByAppendingString:@".obj"];
    cloud_obj_path = [documentsDirectoryPath stringByAppendingPathComponent:cloud_obj];
    
    //Log debug
    NSString *str = [NSString stringWithFormat: @"[IOS INFO] Object cloud found : %@ \n", cloud_obj_path];
    [self writeToTextView:self.debugLogTextField string:str erase:NO];
    
    //Check if the file exist
    BOOL fileExists = [[NSFileManager defaultManager] fileExistsAtPath:cloud_obj_path];
    if (!fileExists){
        [self popupMessage:@"The cloud does not exist" title:@"Error"];
        return;
        
    }
    
    // enable the network activity indicator on the status bar to indicate to the user that a network process is running.
    [[UIApplication sharedApplication] setNetworkActivityIndicatorVisible:YES];
    
    
    //Transform the data to NSData for sending to the server
    NSData *cloudData = [NSData dataWithContentsOfFile:cloud_obj_path];
    
    //Create a HTTP request type POST for sending the data
    NSString *uploadHTTPLink = [NSString stringWithFormat:@"%@%@", mBaseURL, mUpload];
    NSMutableURLRequest *request = [[AFHTTPRequestSerializer serializer] multipartFormRequestWithMethod:@"POST" URLString:uploadHTTPLink parameters:params constructingBodyWithBlock:^(id<AFMultipartFormData>  _Nonnull formData) {
        /// Add attributes to the forms for using then in NodeJS
        [formData appendPartWithFileData:cloudData name:@"cloud" fileName:cloud_obj_path mimeType:@"text/plain"];
        
    } error:nil];
    
    
    [request setCachePolicy:NSURLRequestReloadIgnoringLocalCacheData];
    [request setHTTPShouldHandleCookies:NO];
    [request setTimeoutInterval:20];
    
    AFURLSessionManager *manager = [[AFURLSessionManager alloc] initWithSessionConfiguration:[NSURLSessionConfiguration defaultSessionConfiguration]];
    
    
    manager.responseSerializer = [AFHTTPResponseSerializer serializer];
    
    [manager setTaskDidSendBodyDataBlock:^(NSURLSession *session, NSURLSessionTask *task, int64_t bytesSent, int64_t totalBytesSent, int64_t totalBytesExpectedToSend) {
        //during the progress
        CGFloat progress_percent = (((CGFloat)totalBytesSent / (CGFloat)totalBytesExpectedToSend* 1.0f)*100);
        CGFloat progress = (((CGFloat)totalBytesSent / (CGFloat)totalBytesExpectedToSend* 1.0f));
        
        CGFloat kilobyte_totalSent = totalBytesSent / 1024;
        CGFloat kilobyte_totalExpectedToSent = totalBytesExpectedToSend / 1024;
        CGFloat megabyte_totalSent = kilobyte_totalSent / 1024;
        CGFloat megabyte_totalExpectedToSent = kilobyte_totalExpectedToSent / 1024;
        
        
        NSLog(@"Uploading files %f MO -- > %f MO",megabyte_totalSent,megabyte_totalExpectedToSent);
        NSLog(@"Progress %f %%", progress_percent);
        [m_overlayIndicator setProgress:progress animated:YES];
        
        
    }];
    //Upload it
    //Upload tasks are used for making HTTP requests that require a request body
    NSURLSessionDataTask *uploadTask = [manager dataTaskWithRequest:request completionHandler:^(NSURLResponse *response, id responseObject, NSError *error) {
        
        dispatch_async(dispatch_get_main_queue(), ^{
            [[UIApplication sharedApplication] setNetworkActivityIndicatorVisible:NO];
        });
        
        NSHTTPURLResponse *httpResponse = (NSHTTPURLResponse *)response;
        
        switch (httpResponse.statusCode) {
                /* Code in case of success == Server returned True  */
            case 200:
            {
                NSLog(@"Success");
                
                m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Succeed";
                }
                [m_overlayIndicator dismiss:YES];
                [m_spinner stopAnimating];
                [self downloadResultFromServer];
                
                
                
                //[MRProgressOverlayView dismissOverlayForView:self.view animated:YES];
                break;
            }
                /* Code in case of error == Server returned False  */
            case 420:
            {
                NSLog(@"Error");
                NSLog(@"HTTP code %ld", (long)httpResponse.statusCode);
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];
                [m_spinner stopAnimating];
                
                
                break;
            }
            default:{
                NSLog(@"Defaut Error");
                NSLog(@"HTTP code %ld", (long)httpResponse.statusCode);
                m_overlayIndicator.mode = MRProgressOverlayViewModeCross;
                if (m_overlayIndicator.titleLabel.text.length > 0) {
                    m_overlayIndicator.titleLabelText = @"Failed";
                }
                [m_overlayIndicator dismiss:YES];
                
                [m_spinner stopAnimating];
                break;
            }
        }
        
        
    }];
    [manager uploadProgressForTask:uploadTask];
    //All tasks start in a suspended state by default; calling resume starts the data task
    [uploadTask resume];
    
    
    
}

- (void) checkGlobalConnexion {
    NSLog(@"CHECK CONNEXION");
    //Check connectivity
    [Utils checkBasicConnexion:^(BOOL internet)
     {
         
         if (internet)
         {
             m_isConnectedToInternet = YES;
         }
         else
         {
             m_isConnectedToInternet = NO;
         }
     }];
    [Utils checkServerConnection:^(BOOL server)
     {
         if (server)
         {
             m_isConnectedToServer = YES;
         }
         else
         {
             m_isConnectedToServer = NO;
         }
     }];
    
}

/*!
 @brief When the user click on get results button
 */
- (IBAction)onGetResultsClick:(id)sender {
    [self downloadResultFromServer];
}


- (IBAction)onParametersClick:(id)sender {
    [self presentParametersViewcontroller];
    
}

- (IBAction)onConnexionClick:(id)sender {
    m_overlayIndicator= [MRProgressOverlayView showOverlayAddedTo:self.view animated:YES];
    m_overlayIndicator.mode = MRProgressOverlayViewModeIndeterminate;
    m_overlayIndicator.titleLabelText = @"Checking connectivity...";
    
    [self checkGlobalConnexion];
    
    if (m_isConnectedToServer == YES && m_isConnectedToInternet == YES){
        [self popupMessage:@"Basic connexion and server connexion are okey" title:@"Ok"];
        self.connexionButton.backgroundColor = [Utils colorGREEN];
        [self.connexionButton setTitle:@"Connexion OK" forState:UIControlStateNormal];
        m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
        if (m_overlayIndicator.titleLabel.text.length > 0) {
            m_overlayIndicator.titleLabelText = @"Connexion OK";
        }
        [m_overlayIndicator dismiss:YES];
    }else if (m_isConnectedToInternet == YES && m_isConnectedToServer == NO){
        [self popupMessage:@"Basic connexion OK - Connexion server : failed" title:@"Error"];
        self.connexionButton.backgroundColor = [Utils colorRED];
        [self.connexionButton setTitle:@"Connexion failed" forState:UIControlStateNormal];
        m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
        if (m_overlayIndicator.titleLabel.text.length > 0) {
            m_overlayIndicator.titleLabelText = @"Connexion Failed";
        }
        [m_overlayIndicator dismiss:YES];
    }else {
        
        [self popupMessage:@"No connexion " title:@"Error"];
        self.connexionButton.backgroundColor = [Utils colorRED];
        [self.connexionButton setTitle:@"Connexion failed" forState:UIControlStateNormal];
        m_overlayIndicator.mode = MRProgressOverlayViewModeCheckmark;
        if (m_overlayIndicator.titleLabel.text.length > 0) {
            m_overlayIndicator.titleLabelText = @"Connexion Failed";
        }
        [m_overlayIndicator dismiss:YES];
    }
    
    
}

- (IBAction)methodClassificationOnSwitch:(id)sender {
    
    if (self.methodClassificationControl.selectedSegmentIndex == 0){
        //SVM method
        m_method_search_selected = @"svm";
        self.methodChoseLabel.text = @"SVM";
        self.resultImageCategoryImageView.hidden = NO;
        self.categoryResultSvmLabel.hidden = NO;
        self.classificationResultTableView.hidden = YES;
        
    }
    else {
        //Similarity Search
        m_method_search_selected = @"similaritysearch";
        self.methodChoseLabel.text = @"Similarity Search";
        self.resultImageCategoryImageView.hidden = YES;
        self.categoryResultSvmLabel.hidden = YES;
        self.classificationResultTableView.hidden = NO;
    }
}

- (IBAction)descriptorsClassificationOnSwitch:(id)sender {
    if (self.methodClassificationControl.selectedSegmentIndex == 0 && self.descriptorsClassificationControl.selectedSegmentIndex == 0){
        if (svm_esf_available){
            m_type_descriptor_selected = @"esf";
            self.descriptorChoseLabel.text = @"ESF";
            
        }else {
            [self popupMessage:@"ESF Model not available according to the user" title:@"Error"];
        }
    }else if (self.methodClassificationControl.selectedSegmentIndex == 0 && self.descriptorsClassificationControl.selectedSegmentIndex == 1){
        if (svm_cvfh_available){
            m_type_descriptor_selected = @"cvfh";
            self.descriptorChoseLabel.text = @"CVFH";
        }else {
            [self popupMessage:@"CVFH Model not available according to the user" title:@"Error"];
        }
    }
    else if (self.methodClassificationControl.selectedSegmentIndex == 0 && self.descriptorsClassificationControl.selectedSegmentIndex == 2){
        if (svm_vfh_available){
            m_type_descriptor_selected = @"vfh";
            self.descriptorChoseLabel.text = @"VFH";
        }
        else {
            [self popupMessage:@"VFH Model not available according to the user" title:@"Error"];
        }
    }
    if (self.methodClassificationControl.selectedSegmentIndex == 0 && self.descriptorsClassificationControl.selectedSegmentIndex == 3){
    
        if (svm_ourcvfh_available){
            m_type_descriptor_selected = @"ourcvfh";
            self.descriptorChoseLabel.text = @"OUR-CVFH";
        }
        else {
            [self popupMessage:@"OURCVFH Model not available according to the user" title:@"Error"];
        }
    }
    
}

- (IBAction)datasetClassificationOnSwitch:(id)sender {
    if (self.datasetClassificationControl.selectedSegmentIndex == 0){
        //Structure Sensor Dataset
        m_type_dataset_selected = @"structure";
        self.datasetTypeLabel.text = @"Structure Sensor";
         categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_structure"];
    }
    else if (self.datasetClassificationControl.selectedSegmentIndex == 1){
        m_type_dataset_selected = @"pottery";
        self.datasetTypeLabel.text = @"Pottery";
        categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_pottery"];
    }
    else if (self.datasetClassificationControl.selectedSegmentIndex == 2){
        m_type_dataset_selected = @"potterymix";
        self.datasetTypeLabel.text = @"Pottery Mix";
         categories_arr = [self loadCategoriesFromJasonFileCategories:@"categories_potterymix"];
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


@end
