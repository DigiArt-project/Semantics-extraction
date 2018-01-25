//
//  Utils.h
//  Visual_Search
//
//  Created by Lirone Samoun on 20/01/2017.
//  Copyright © 2017 Occipital. All rights reserved.
//

#ifndef Utils_h
#define Utils_h

//IP unice hotspot
//static NSString* const mBaseURL = @"http://10.212.102.247:8080/";
//IP server i3S
static NSString* const mBaseURL = @"http://134.59.130.158:8080/";
static NSString* const mUpload = @"api/upload/action_upload";
static NSString* const mDownload = @"api/results";
static NSString* const mParametersUrl = @"utilities/";
//Localhost with simulator
//static NSString* const kBaseURL = @"http://localhost:3000/upload/";



typedef void(^connection)(BOOL);

@interface Utils : NSObject

+ (BOOL)checkBasicConnexion:(connection)block;
+ (void)checkServerConnection:(connection)block;
+ (UIColor *)colorRED;
+ (UIColor *)colorGREEN;
+ (UIColor *)colorBLUE;
+ (UIColor *)colorBLACK;


@end




#endif /* Utils_h */
