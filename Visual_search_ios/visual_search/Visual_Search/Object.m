//
//  Object.m
//  Visual_Search
//
//  Created by Lirone Samoun on 18/01/2017.
//  Copyright © 2017 Occipital. All rights reserved.
//

#import "Object.h"

#define safeSet(d,k,v) if (v) d[k] = v;

@implementation Object

- (instancetype) init
{
    self = [super init];
 
    return self;
}
- (id)initWithName:(NSString*)name id:(NSString*)ido description:(NSString*)description category:(Category*)category{
    self = [super init];
    if (self)
    {
        _m_id = ido;
        _m_name = name;
        _m_description = description;
        _m_category = category;
        
    }
    return self;
}

#pragma mark - serialization

- (instancetype) initWithDictionary:(NSDictionary*)dictionary
{
    self = [super init];
    if (self) {
        _m_name = dictionary[@"name"];
        _m_description = dictionary[@"description"];
        _m_id = dictionary[@"id"];
    }
    return self;
}

- (NSDictionary*) toDictionary
{
    NSMutableDictionary* jsonable = [NSMutableDictionary dictionary];
    safeSet(jsonable, @"name", self.m_name);
    safeSet(jsonable, @"description", self.m_description);
    safeSet(jsonable, @"_id", self.m_id);
    return jsonable;
}

@end
