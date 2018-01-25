//
//  Category.m
//  Visual_Search
//
//  Created by Lirone Samoun on 18/01/2017.
//  Copyright © 2017 Occipital. All rights reserved.
//

#import "Category.h"

#define safeSet(d,k,v) if (v) d[k] = v;

@implementation Category


- (instancetype) init
{
    self = [super init];
    return self;
}

- (id)initWithName:(NSString*)name description:(NSString*)description cover:(UIImage*)image{
    self = [super init];
    if (self)
    {
        _m_name = name;
        _m_description = description;
        _m_cover = image;
    }
    return self;
}


#pragma mark - serialization



- (instancetype) initWithDictionary:(NSDictionary*)dictionary
{
    self = [super init];
    if (self) {
        _m_name = dictionary[@"name"];
        _m_cover = dictionary[@"image"];
        _m_description = dictionary[@"description"];
    }
    return self;
}

- (NSDictionary*) toDictionary
{
    NSMutableDictionary* jsonable = [NSMutableDictionary dictionary];
    safeSet(jsonable, @"name", self.m_name);
    safeSet(jsonable, @"image", self.m_cover);
    safeSet(jsonable, @"description", self.m_description);
    return jsonable;
}

@end
