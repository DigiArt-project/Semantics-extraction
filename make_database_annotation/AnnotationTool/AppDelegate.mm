/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import "AppDelegate.h"
#import "ViewController.h"

//------------------------------------------------------------------------------

namespace {

void preventApplicationFromStartingInTheBackgroundWhenTheStructureSensorIsPlugged ()
{
    // Sadly, iOS 9.2+ introduced unexpected behavior: every time a Structure Sensor is plugged in to iOS, iOS will launch all Structure-related apps in the background.
    // The apps will not be visible to the user.
    // This can cause problems since Structure SDK apps typically ask the user for permission to use the camera when launched.
    // This leads to the user's first experience with a Structure SDK app being:
    //     1.  Download Structure SDK apps from App Store.
    //     2.  Plug in Structure Sensor to iPad.
    //     3.  Get bombarded with "X app wants to use the Camera" notifications from every installed Structure SDK app.
    // Each app has to deal with this problem in its own way.
    // In the Structure SDK, sample apps peacefully exit without causing a crash report.
    // This also has other benefits, such as not using memory.
    // Note that Structure SDK does not support connecting to Structure Sensor if the app is in the background.

    if ([UIApplication sharedApplication].applicationState == UIApplicationStateBackground)
    {
        NSLog(@"iOS launched %@ in the background. This app is not designed to be launched in the background, so it will exit peacefully.",
            [[NSBundle mainBundle] objectForInfoDictionaryKey:@"CFBundleDisplayName"]
        );

        exit(0);
    }
}

}

//------------------------------------------------------------------------------

@implementation AppDelegate

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions
{
    preventApplicationFromStartingInTheBackgroundWhenTheStructureSensorIsPlugged();

    if (false)
    {
        // STWirelessLog is very helpful for debugging while your Structure Sensor is plugged in.
        // See SDK documentation for how to start a listener on your computer.

        NSError* error = nil;
        NSString *remoteLogHost = @"192.168.1.1";
        [STWirelessLog broadcastLogsToWirelessConsoleAtAddress:remoteLogHost usingPort:4999 error:&error];
        if (error)
            NSLog(@"Oh no! Can't start wireless log: %@", [error localizedDescription]);
    }
    
    self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
    // Override point for customization after application launch.
    if ([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPhone) {
        self.viewController = [[ViewController alloc] initWithNibName:@"ViewController_iPhone" bundle:nil];
    } else {
        self.viewController = [[ViewController alloc] initWithNibName:@"ViewController_iPad" bundle:nil];
    }
    self.window.rootViewController = self.viewController;
    [self.window makeKeyAndVisible];
    
    return YES;
}

- (void)applicationWillResignActive:(UIApplication *)application
{
    // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
    // Use this method to pause ongoing tasks, disable timers, and throttle down OpenGL ES frame rates. Games should use this method to pause the game.
}

- (void)applicationDidEnterBackground:(UIApplication *)application
{
    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later. 
    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
}

- (void)applicationWillEnterForeground:(UIApplication *)application
{
    // Called as part of the transition from the background to the inactive state; here you can undo many of the changes made on entering the background.
}

- (void)applicationDidBecomeActive:(UIApplication *)application
{
    // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
}

- (void)applicationWillTerminate:(UIApplication *)application
{
    // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
}

@end
