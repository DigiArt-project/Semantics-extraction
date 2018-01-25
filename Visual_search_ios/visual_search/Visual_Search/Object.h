/*!
 @class Object
 
 @brief class for describing an object found by classification

 */

#import <Foundation/Foundation.h>
#import "Category.h"

@interface Object : NSObject

/*! @brief id of the object */
@property (nonatomic, copy) NSString* m_id;
/*! @brief name of the object. */
@property (nonatomic, copy) NSString* m_name;
/*! @brief A description of the object */
@property (nonatomic, copy) NSString* m_description;
/*! @brief The category which belongs to the object */
@property (nonatomic, copy) Category* m_category;


/*!
 @brief Init the object instance
 
 @param ido the id of the object
        name The name of the object
        description The description of the object
        category The category of the object
 @return the instance
 */
- (id)initWithName:(NSString*)name id:(NSString*)ido description:(NSString*)description category:(Category*)category ;

#pragma mark - JSON-ification

- (instancetype) initWithDictionary:(NSDictionary*)dictionary;
- (NSDictionary*) toDictionary;

@end
