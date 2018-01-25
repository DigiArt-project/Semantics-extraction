//
//  Category.h
//  Visual_Search
//
//  Created by Lirone Samoun on 18/01/2017.
//

/*!
 @class Category
 
 @brief Class which describe a category
 
 */

#import <Foundation/Foundation.h>

@interface Category : NSObject

/*! @brief Name of the category. */
@property (nonatomic, copy) NSString* m_name;
/*! @brief Description of the category. */
@property (nonatomic, copy) NSString* m_description;
/* ! image cover of the category */
@property (nonatomic, copy) UIImage* m_cover;

/*!
 @brief Init the category instance
 
 @param  name The name of the category
         description The description of the category
         image The cover image of the category
 @return the instance
 */
- (id)initWithName:(NSString*)name description:(NSString*)description cover:(UIImage*)image;

#pragma mark - JSON-ification

- (instancetype) initWithDictionary:(NSDictionary*)dictionary;
- (NSDictionary*) toDictionary;



@end
