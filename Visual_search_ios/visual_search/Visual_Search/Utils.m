//
//  Utils.m
//  Visual_Search
//
//  Created by Lirone Samoun on 02/02/2017.
//  Copyright © 2017 Occipital. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "Utils.h"

@implementation Utils

+ (BOOL)checkBasicConnexion:(connection)block
{
    NSURL *url = [NSURL URLWithString:@"http://www.google.com"];
    NSMutableURLRequest *headRequest = [NSMutableURLRequest requestWithURL:url];
    headRequest.HTTPMethod = @"HEAD";
    
    NSURLSessionConfiguration *defaultConfigObject = [NSURLSessionConfiguration ephemeralSessionConfiguration];
    defaultConfigObject.timeoutIntervalForResource = 10.0;
    defaultConfigObject.requestCachePolicy = NSURLRequestReloadIgnoringLocalAndRemoteCacheData;
    
    NSURLSession *defaultSession = [NSURLSession sessionWithConfiguration:defaultConfigObject delegate:self delegateQueue: [NSOperationQueue mainQueue]];
    
    NSURLSessionDataTask *dataTask = [defaultSession dataTaskWithRequest:headRequest
                                                       completionHandler:^(NSData *data, NSURLResponse *response, NSError *error)
                                      {
                                          if (!error && response)
                                          {
                                              block([(NSHTTPURLResponse *)response statusCode] == 200);
                                          }else {
                                              block(NO);
                                          }
                                      }];
    [dataTask resume];
}

+ (void)checkServerConnection:(connection)block
{
    NSURL *url = [NSURL URLWithString:mBaseURL];
    NSMutableURLRequest *headRequest = [NSMutableURLRequest requestWithURL:url];
    headRequest.HTTPMethod = @"HEAD";
    
    NSURLSessionConfiguration *defaultConfigObject = [NSURLSessionConfiguration ephemeralSessionConfiguration];
    defaultConfigObject.timeoutIntervalForResource = 10.0;
    defaultConfigObject.requestCachePolicy = NSURLRequestReloadIgnoringLocalAndRemoteCacheData;
    
    NSURLSession *defaultSession = [NSURLSession sessionWithConfiguration:defaultConfigObject delegate:self delegateQueue: [NSOperationQueue mainQueue]];
    
    NSURLSessionDataTask *dataTask = [defaultSession dataTaskWithRequest:headRequest
                                                       completionHandler:^(NSData *data, NSURLResponse *response, NSError *error)
                                      {
                                          if (!error && response)
                                          {
                                              block([(NSHTTPURLResponse *)response statusCode] == 200);
                                          }else {
                                              block(NO);
                                          }
                                      }];
    [dataTask resume];
}

+ (UIColor *)colorRED
{
    static UIColor* colorRED = nil;
    if (colorRED == nil)
    {
        colorRED = [[UIColor alloc]initWithRed:255/255.0 green:102/255.0 blue:102.0/255.0 alpha:0.6];
    }
    return colorRED;
}

+ (UIColor *)colorGREEN
{
    static UIColor* colorGREEN = nil;
    if (colorGREEN == nil)
    {
        colorGREEN = [[UIColor alloc]initWithRed:102/255.0 green:204/255.0 blue:0/255.0 alpha:0.6];
    }
    return colorGREEN;
}

+ (UIColor *)colorBLUE
{
    static UIColor* colorBLUE = nil;
    if (colorBLUE == nil)
    {
        colorBLUE = [[UIColor alloc]initWithRed:0/255.0 green:51/255.0 blue:102.0/255.0 alpha:0.6];
    }
    return colorBLUE;
}

+ (UIColor *)colorBLACK
{
    static UIColor* colorBLACK = nil;
    if (colorBLACK == nil)
    {
        colorBLACK = [[UIColor alloc]initWithRed:0/255.0 green:0/255.0 blue:0/255.0 alpha:1];
    }
    return colorBLACK;
}
@end

